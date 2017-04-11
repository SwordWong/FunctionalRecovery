#include "WarpField.h"
#include "TsdfVolume.h"
#include "GpuMesh.h"
#include "GpuKdTree.h"
#include "ldp_basic_mat.h"
#include "ldpMat\Quaternion.h"
namespace dfusion
{
	WarpField::WarpField()
	{
		//m_volume = nullptr;
		m_knnField = nullptr;
		for (int k = 0; k < GraphLevelNum; k++)
			m_nodeTree[k] = nullptr;
		m_current_point_buffer_size = 0;
		memset(m_numNodes, 0, sizeof(m_numNodes));
		memset(m_lastNumNodes, 0, sizeof(m_lastNumNodes));
		m_activeVisualizeNodeId = -1;
		m_knnFieldSurface = 0;
		m_knnFieldTexture = 0;
		m_nodesDqVeTexture = 0;
	}

	WarpField::~WarpField()
	{
		clear();
	}

	//void WarpField::init(TsdfVolume* vol, Param param)
	void WarpField::init( Param param, WarpVolumeParam wvParam)
	{
		m_param = param;
		//m_volume = vol;
		m_rigidTransform = Tbx::Transfo::identity();
		m_nodesQuatTransVw.create(MaxNodeNum*GraphLevelNum * 3);
		m_nodesGraph.create(MaxNodeNum*GraphLevelNum);
		cudaMemset(m_nodesQuatTransVw.ptr(), 0, m_nodesQuatTransVw.size()*m_nodesQuatTransVw.elem_size);
		cudaMemset(m_nodesGraph.ptr(), 0, m_nodesGraph.size()*m_nodesGraph.elem_size);
		unBindNodesDqVwTexture();
		bindNodesDqVwTexture();
		for (int lv = 0; lv < GraphLevelNum; lv++)
			m_numNodes[lv] = 0;
		printf("WarpField::init: warp volume res = (%d,%d,%d)\n", wvParam.warp_volume_res.x, wvParam.warp_volume_res.y, wvParam.warp_volume_res.z);
		/*wvParam.warp_volume_res.x = m_param.volume_resolution[0];
		wvParam.warp_volume_res.y = m_param.volume_resolution[1];
		wvParam.warp_volume_res.z = m_param.volume_resolution[2];
		wvParam.warp_voxel_size = 1.0f/m_param.voxels_per_meter;
		wvParam.warp_volume_original = make_float3(-(m_param.volume_resolution[0] - 1)*0.5f / m_param.voxels_per_meter,
			-(m_param.volume_resolution[1] - 1) * 0.5f / m_param.voxels_per_meter,
			-KINECT_NEAREST_METER - float(m_param.volume_resolution[2] - 1) / m_param.voxels_per_meter);*/
		int3 res = wvParam.warp_volume_res;
		float vsz = wvParam.warp_voxel_size;
		warp_volume_param = wvParam;
		// knn volume
		// malloc 3D texture
		if (m_knnField)
			cudaSafeCall(cudaFreeArray(m_knnField), "cudaFreeArray");
		cudaExtent ext = make_cudaExtent(res.x, res.y, res.z);
		cudaChannelFormatDesc desc = cudaCreateChannelDesc<KnnIdx>();
		cudaSafeCall(cudaMalloc3DArray(&m_knnField, &desc, ext), "WarpField::init, cudaMalloc3D");
		unBindKnnFieldSurface();
		unBindKnnFieldTexture();
		bindKnnFieldSurface();
		bindKnnFieldTexture();
		initKnnField();

		for (int k = 0; k < GraphLevelNum; k++)
		{
			if (m_nodeTree[k])
				delete m_nodeTree[k];
			m_nodeTree[k] = new GpuKdTree();
		}

		// nodes grid
		m_nodesGridSize = make_int3(std::ceil(res.x*vsz/m_param.warp_radius_search_epsilon),
			std::ceil(res.y*vsz/m_param.warp_radius_search_epsilon),
			std::ceil(res.z*vsz/m_param.warp_radius_search_epsilon));
	}

	void WarpField::clear()
	{
		unBindNodesDqVwTexture();
		unBindKnnFieldSurface();
		unBindKnnFieldTexture();
		for (int k = 0; k < GraphLevelNum; k++)
		{
			if (m_nodeTree[k])
			{
				delete m_nodeTree[k];
				m_nodeTree[k] = nullptr;
			}
		}
		if (m_knnField)
		{
			cudaSafeCall(cudaFreeArray(m_knnField), "cudaFreeArray");
			m_knnField = nullptr;
		}
		m_nodesQuatTransVw.release();
		m_nodesGraph.release();

		m_meshPointsSorted.release();
		m_meshPointsKey.release();
		m_meshPointsFlags.release();
		m_tmpBuffer.release();

		m_current_point_buffer_size = 0;
		memset(m_numNodes, 0, sizeof(m_numNodes));
		memset(m_lastNumNodes, 0, sizeof(m_lastNumNodes));
		m_activeVisualizeNodeId = -1;
	}

	cudaSurfaceObject_t WarpField::getKnnFieldSurface()const
	{
		return m_knnFieldSurface;
	}

	cudaTextureObject_t WarpField::getKnnFieldTexture()const
	{
		return m_knnFieldTexture;
	}

	cudaTextureObject_t WarpField::getNodesDqVwTexture()const
	{
		return m_nodesDqVeTexture;
	}

	void WarpField::updateWarpNodes(GpuMesh& src)
	{
		memcpy(m_lastNumNodes, m_numNodes, sizeof(m_numNodes));

		insertNewNodes(src);

		if (m_param.graph_single_level)
		{
			updateGraph_singleLevel();
		}
		else
		{
			for (int lv = 1; lv < GraphLevelNum; lv++)
				updateGraph(lv);
		}

		remove_small_graph_components();

		if (m_lastNumNodes[0] != m_numNodes[0])
			updateAnnField();
	}

	void WarpField::bindKnnFieldSurface()
	{
		cudaResourceDesc    surfRes;
		memset(&surfRes, 0, sizeof(cudaResourceDesc));
		surfRes.resType = cudaResourceTypeArray;
		surfRes.res.array.array = m_knnField;
		cudaSafeCall(cudaCreateSurfaceObject(&m_knnFieldSurface, &surfRes),
			"WarpField::bindKnnFieldSurface");
	}

	void WarpField::unBindKnnFieldSurface()
	{
		cudaSafeCall(cudaDestroySurfaceObject(m_knnFieldSurface),
			"WarpField::unBindKnnFieldSurface");
	}

	void WarpField::bindKnnFieldTexture()
	{
		cudaResourceDesc texRes;
		memset(&texRes, 0, sizeof(cudaResourceDesc));
		texRes.resType = cudaResourceTypeArray;
		texRes.res.array.array = m_knnField;
		cudaTextureDesc texDescr;
		memset(&texDescr, 0, sizeof(cudaTextureDesc));
		texDescr.normalizedCoords = 0;
		texDescr.filterMode = cudaFilterModePoint;
		texDescr.addressMode[0] = cudaAddressModeClamp;
		texDescr.addressMode[1] = cudaAddressModeClamp;
		texDescr.addressMode[2] = cudaAddressModeClamp;
		texDescr.readMode = cudaReadModeElementType;
		cudaSafeCall(cudaCreateTextureObject(&m_knnFieldTexture, &texRes, &texDescr, NULL),
			"WarpField::bindKnnFieldTexture");
	}

	void WarpField::unBindKnnFieldTexture()
	{
		cudaSafeCall(cudaDestroyTextureObject(m_knnFieldTexture),
			"WarpField::unBindKnnFieldTexture");
	}

	void WarpField::bindNodesDqVwTexture()
	{
		cudaResourceDesc texRes;
		memset(&texRes, 0, sizeof(cudaResourceDesc));
		texRes.resType = cudaResourceTypeLinear;
		texRes.res.linear.devPtr = (void*)m_nodesQuatTransVw.ptr();
		texRes.res.linear.desc = cudaCreateChannelDesc<float4>();
		texRes.res.linear.sizeInBytes = m_nodesQuatTransVw.size()*sizeof(float4);
		cudaTextureDesc texDescr;
		memset(&texDescr, 0, sizeof(cudaTextureDesc));
		texDescr.normalizedCoords = 0;
		texDescr.filterMode = cudaFilterModePoint;
		texDescr.addressMode[0] = cudaAddressModeClamp;
		texDescr.addressMode[1] = cudaAddressModeClamp;
		texDescr.addressMode[2] = cudaAddressModeClamp;
		texDescr.readMode = cudaReadModeElementType;
		cudaSafeCall(cudaCreateTextureObject(&m_nodesDqVeTexture, &texRes, &texDescr, NULL),
			"WarpField::bindNodesDqVwTexture");
	}

	void WarpField::unBindNodesDqVwTexture()
	{
		cudaSafeCall(cudaDestroyTextureObject(m_nodesDqVeTexture),
			"WarpField::unBindNodesDqVwTexture");
	}

	/*const TsdfVolume* WarpField::getVolume()const
	{
		return m_volume;
	}*/

	void WarpField::setActiveVisualizeNodeId(int id)
	{
		m_activeVisualizeNodeId = min(getNumNodesInLevel(0)-1, id);
	}

	int WarpField::getActiveVisualizeNodeId()const
	{
		return m_activeVisualizeNodeId;
	}

	void WarpField::save(const char* filename)const
	{
		FILE* pFile = fopen(filename, "wb");
		if (!pFile)
			throw std::exception(("save failed" + std::string(filename)).c_str());

		fwrite(&m_rigidTransform, sizeof(m_rigidTransform), 1, pFile);
		fwrite(m_numNodes, sizeof(m_numNodes), 1, pFile);

		std::vector<float4> tmp;
		m_nodesQuatTransVw.download(tmp);
		int ntmp = tmp.size();
		fwrite(&ntmp, sizeof(int), 1, pFile);
		fwrite(tmp.data(), sizeof(float4), tmp.size(), pFile);

		std::vector<KnnIdx> tmpIdx;
		m_nodesGraph.download(tmpIdx);
		int ntmpidx = tmpIdx.size();
		fwrite(&ntmpidx, sizeof(int), 1, pFile);
		fwrite(tmpIdx.data(), sizeof(KnnIdx), tmpIdx.size(), pFile);

		fclose(pFile);
	}

	void WarpField::load(const char* filename)
	{
		/*if (m_volume == nullptr)
			throw std::exception("Error: not initialzied before loading WarpField!");*/

		/*init(m_volume, m_param);

		FILE* pFile = fopen(filename, "rb");
		if (!pFile)
			throw std::exception(("load failed" + std::string(filename)).c_str());
	
		memset(m_lastNumNodes, 0, sizeof(m_lastNumNodes));
		fread(&m_rigidTransform, sizeof(m_rigidTransform), 1, pFile);
		fread(m_numNodes, sizeof(m_numNodes), 1, pFile);
		
		std::vector<float4> tmp(m_nodesQuatTransVw.size(), make_float4(0.f,0.f,0.f,0.f));
		int ntmp = 0;
		fread(&ntmp, sizeof(int), 1, pFile);
		if (ntmp != tmp.size())
			throw std::exception("size not matched in WarpField::load nodesQuatTransVw");
		fread(tmp.data(), sizeof(float4), tmp.size(), pFile);
		m_nodesQuatTransVw.upload(tmp);		
		unBindNodesDqVwTexture();
		bindNodesDqVwTexture();

		std::vector<KnnIdx> tmpIdx(m_nodesGraph.size(), make_knn(0));
		int ntmpidx = 0;
		fread(&ntmpidx, sizeof(int), 1, pFile);
		if (ntmpidx != tmpIdx.size())
			throw std::exception("size not matched in WarpField::load nodesGraph");
		fread(tmpIdx.data(), sizeof(KnnIdx), tmpIdx.size(), pFile);
		m_nodesGraph.upload(tmpIdx);

		fclose(pFile);

		updateAnnField();

		printf("warp field loaded: %d %d %d %d nodes\n", m_numNodes[0],
			m_numNodes[1], m_numNodes[2], m_numNodes[3]);*/
	}

	static ldp::Mat4f convert(Tbx::Transfo T)
	{
		ldp::Mat4f A;
		for (int y = 0; y < 4; y++)
		for (int x = 0; x < 4; x++)
			A(y, x) = T[y * 4 + x];
		return A;
	}
	inline ldp::DualQuaternionF convert(Tbx::Dual_quat_cu dq)
	{
		Tbx::Quat_cu q0 = dq.get_non_dual_part();
		Tbx::Quat_cu q1 = dq.get_dual_part();
		Tbx::Vec3 q0_v = q0.get_vec_part();
		Tbx::Vec3 q1_v = q1.get_vec_part();

		ldp::DualQuaternionF o;
		o.dq[0].v = ldp::Float3(q0_v.x, q0_v.y, q0_v.z);
		o.dq[1].v = ldp::Float3(q1_v.x, q1_v.y, q1_v.z);
		o.dq[0].w = q0.w();
		o.dq[1].w = q1.w();
		return o;
	}

	void WarpField::export_vertKnn_nodes_graph(GpuMesh& src, FILE* pFile)
	{
		if (!pFile)
			throw std::exception("null file pointer");

		// 0. knn for vertices
		DeviceArray<KnnIdx> knns;
		extract_knnIdx_for_verts(src, knns);
		std::vector<KnnIdx> knnsHost;
		knns.download(knnsHost);
		int nVerts = knnsHost.size();
		fwrite(&nVerts, sizeof(nVerts), 1, pFile);
		int szEle = sizeof(KnnIdx);
		fwrite(&szEle, sizeof(int), 1, pFile);
		fwrite(knnsHost.data(), sizeof(KnnIdx), nVerts, pFile);
		
		// 1. rigid transform
		ldp::Mat4f T = convert(m_rigidTransform);
		fwrite(T.ptr(), sizeof(float), T.nRow()*T.nCol(), pFile);

		// 2. nodes
		int numAllNodes = getNumAllNodes();
		fwrite(&numAllNodes, sizeof(numAllNodes), 1, pFile);
		std::vector<float4> tmp;
		m_nodesQuatTransVw.download(tmp);
		for (int iLevel = 0; iLevel < getNumLevels(); iLevel++)
		{
			const float4* nodesQuatTransVw = tmp.data() + 3 * MaxNodeNum * iLevel;
			for (int iNode = 0; iNode < m_numNodes[iLevel]; iNode++)
			{
				Tbx::Dual_quat_cu dq = pack_dual_quat(nodesQuatTransVw[iNode * 3], nodesQuatTransVw[iNode*3+1]);
				float4 vw = nodesQuatTransVw[iNode * 3 + 2];
				vw.w = 1.f / vw.w; // the vw.w were stored inversely in the GPU, now we reverse it
				ldp::DualQuaternionF ldp_dq = convert(dq);
				ldp::Float4 ldp_vw(vw.x, vw.y, vw.z, vw.w);
				fwrite(&ldp_dq, sizeof(ldp_dq), 1, pFile);
				fwrite(&ldp_vw, sizeof(ldp_vw), 1, pFile);
			} // iNode
		} // iLevel

		// 3. graph
		std::vector<KnnIdx> tmpIdx;
		m_nodesGraph.download(tmpIdx);
		int idxMapper[WarpField::GraphLevelNum+1];
		idxMapper[0] = 0;;
		for (int l = 1; l <= GraphLevelNum; l++)
			idxMapper[l] = idxMapper[l - 1] + m_numNodes[l];
		for (int iLevel = 0; iLevel < getNumLevels(); iLevel++)
		{
			const KnnIdx* knnPtr = tmpIdx.data() + MaxNodeNum * iLevel;
			for (int iNode = 0; iNode < m_numNodes[iLevel]; iNode++)
			{
				KnnIdx kid = knnPtr[iNode];
				for (int k = 0; k < KnnK; k++)
				{
					if (!m_param.graph_single_level)
						knn_k(kid, k) = (knn_k(kid, k) < m_numNodes[iLevel] ?
						knn_k(kid, k) + idxMapper[iLevel] : numAllNodes);
					else
						knn_k(kid, k) = (knn_k(kid, k) < WarpField::MaxNodeNum ?
						knn_k(kid, k) : numAllNodes);
				}

				fwrite(&kid, sizeof(kid), 1, pFile);
			} // iNode
		} // iLevel
	}
}