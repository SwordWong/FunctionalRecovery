#include "WarpProcessor.h"
#include "GpuMesh.h"
#include "RayCaster.h"
//#include "MarchingCubes.h"
#include "TsdfVolume.h"
#include "Camera.h"
#include "WarpField.h"
#include "fmath.h"
#include <eigen\Dense>
#include <eigen\Geometry>
#include "ObjMesh.h"
#include "VolumeData.h"
#include "CpuGaussNewton.h"
#include "GpuGaussNewtonSolver.h"
#include "SparseVolume.h"
#include "RenderedMesh.h"
#include "meshAlignByMarker.h"
#include "RemoveDuplicateVertex.h"
#include "getLargestLinkedSubmesh.h"
namespace dfusion
{
	static void check(const char* msg)
	{
		cudaSafeCall(cudaThreadSynchronize());
		printf("%s\n", msg);
	}

#define WARPPROCESSOR_SAFE_DELETE(buffer)\
	if (buffer){ delete buffer; buffer = nullptr; }

	Tbx::Mat3 convert(Eigen::Matrix3f A)
	{
		return Tbx::Mat3(A(0, 0), A(0, 1), A(0, 2),
			A(1, 0), A(1, 1), A(1, 2),
			A(2, 0), A(2, 1), A(2, 2));
	}
	Tbx::Vec3 convert(Eigen::Vector3f A)
	{
		return Tbx::Vec3(A[0], A[1], A[2]);
	}
	ldp::Mat4f convert(Tbx::Transfo T)
	{
		ldp::Mat4f A;
		for (int y = 0; y < 4; y++)
			for (int x = 0; x < 4; x++)
				A(y, x) = T[y * 4 + x];
		return A;
	}
	WarpProcessor::WarpProcessor()
	{
		m_camera = nullptr;

		m_canoMesh = nullptr;
		m_warpedMesh = nullptr;
		m_warpField = nullptr;
		m_frame_id = 0;
		m_gsSolver = nullptr;
		canoObjMesh = new ObjMesh;
		warpedObjMesh = new ObjMesh;
	}
	WarpProcessor::~WarpProcessor()
	{
		clear();
		WARPPROCESSOR_SAFE_DELETE(m_camera);

		WARPPROCESSOR_SAFE_DELETE(m_canoMesh);
		WARPPROCESSOR_SAFE_DELETE(m_warpedMesh);
		WARPPROCESSOR_SAFE_DELETE(m_warpField);
		WARPPROCESSOR_SAFE_DELETE(m_gsSolver);
	}
	void WarpProcessor::init(dfusion::Param param)
	{
		clear();

		m_param = param;
		// allocation----------------------

		// camera
		if (m_camera == nullptr)
			m_camera = new Camera();
		m_camera->setViewPort(0, dfusion::KINECT_WIDTH, 0, dfusion::KINECT_HEIGHT);
		/*m_camera->setPerspective(KINECT_DEPTH_V_FOV, float(KINECT_WIDTH) / KINECT_HEIGHT,
		KINECT_NEAREST_METER, 30.f);*/
		m_camera->setPerspective(KINECT_DEPTH_V_FOV, float(dfusion::KINECT_WIDTH) / dfusion::KINECT_HEIGHT,
			0.1, 30.f);
		const float l = m_camera->getViewPortLeft();
		const float r = m_camera->getViewPortRight();
		const float t = m_camera->getViewPortTop();
		const float b = m_camera->getViewPortBottom();
		const float	f = (b - t) * 0.5f / tanf(m_camera->getFov() * fmath::DEG_TO_RAD * 0.5f);
		m_kinect_intr = Intr(f, f, (l + r) / 2, (t + b) / 2);

		// volume
		/*if (m_volume == nullptr)
			m_volume = new TsdfVolume();
		m_volume->init(make_int3(
			m_param.volume_resolution[0],
			m_param.volume_resolution[1],
			m_param.volume_resolution[2]),
			1.f / m_param.voxels_per_meter,
			make_float3(-(m_param.volume_resolution[0] - 1)*0.5f / m_param.voxels_per_meter,
				-(m_param.volume_resolution[1] - 1) * 0.5f / m_param.voxels_per_meter,
				-KINECT_NEAREST_METER - float(m_param.volume_resolution[2] - 1) / m_param.voxels_per_meter)
		);*/


		// mesh
		if (m_canoMesh == nullptr)
			m_canoMesh = new GpuMesh();
		if (m_warpedMesh == nullptr)
			m_warpedMesh = new GpuMesh();
		m_canoMesh->setShowColor(m_param.view_show_color);
		m_warpedMesh->setShowColor(m_param.view_show_color);

		

		

		// warp field
		if (m_warpField == nullptr)
			m_warpField = new WarpField();

		//m_warpField->init(m_param);


		// GaussNewton solver
		if (m_gsSolver == nullptr)
			m_gsSolver = new GpuGaussNewtonSolver();

		// maps
		m_depth_curr_pyd.resize(RIGID_ALIGN_PYD_LEVELS);
		m_vmap_curr_pyd.resize(RIGID_ALIGN_PYD_LEVELS);
		m_nmap_curr_pyd.resize(RIGID_ALIGN_PYD_LEVELS);
		m_depth_prev_pyd.resize(RIGID_ALIGN_PYD_LEVELS);
		m_vmap_prev_pyd.resize(RIGID_ALIGN_PYD_LEVELS);
		m_nmap_prev_pyd.resize(RIGID_ALIGN_PYD_LEVELS);
		m_vmap_cano.create(KINECT_HEIGHT, KINECT_WIDTH);
		m_nmap_cano.create(KINECT_HEIGHT, KINECT_WIDTH);
		m_vmap_warp.create(KINECT_HEIGHT, KINECT_WIDTH);
		m_nmap_warp.create(KINECT_HEIGHT, KINECT_WIDTH);

		// finally reset----------------------
		reset();
	}
	void WarpProcessor::clear()
	{
		if (m_camera)
			m_camera->reset();
	
		if (m_canoMesh)
			m_canoMesh->release();
		if (m_warpedMesh)
			m_warpedMesh->release();
		if (m_warpField)
			m_warpField->clear();
		if (m_gsSolver)
			m_gsSolver->reset();
		m_frame_id = 0;
		m_depth_input.release();
		m_depth_curr_pyd.clear();
		m_vmap_curr_pyd.clear();
		m_nmap_curr_pyd.clear();
		m_depth_prev_pyd.clear();
		m_vmap_prev_pyd.clear();
		m_nmap_prev_pyd.clear();
		m_vmap_cano.release();
		m_nmap_cano.release();
		m_vmap_warp.release();
		m_nmap_warp.release();
		m_rigid_sumbuf.release();
		m_rigid_gbuf.release();
	}
	void WarpProcessor::reset()
	{
		// camera
		m_camera->setModelViewMatrix(ldp::Mat4f().eye());

		
		// mesh
		m_canoMesh->release();
		m_warpedMesh->release();

		// warp fields
		//m_warpField->init(m_volume, m_param);

		// solver
		m_gsSolver->reset();

		m_frame_id = 0;
	}
	void WarpProcessor::processFrame(const dfusion::DepthMap & depth, const dfusion::ColorMap & color)
	{
		depth.copyTo(m_depth_input);

		color.copyTo(m_color_input);
		//eroseColor(color, m_color_input, 1);

		Tbx::Transfo rigid = rigid_align();
		rigid_transform = rigid;
		m_warpField->set_rigidTransform(rigid);
		estimateWarpField();
		
		surfaceWarpping();
		insertNewDeformNodes();

		m_frame_id++;
	}

	void WarpProcessor::shading(const Camera & userCam, dfusion::LightSource light, dfusion::ColorMap & img, bool use_ray_casting)
	{
		
		
		// read passed image pos and return selections/related-knns for visualization
		static int lastX = -1, lastY = -1;
		static KnnIdx lastKnn;
		static float3 lastCano;
		const int x = m_param.view_click_vert_xy[0];
		const int y = m_param.view_click_vert_xy[1];
		KnnIdx* knnPtr = nullptr;
		KnnIdx knnIdx = make_knn(WarpField::MaxNodeNum);
		float3* canoPosPtr = nullptr;
		float3 canoPos = make_float3(0, 0, 0);
		if (lastX != x || lastY != y)
		{
			lastX = x;
			lastY = y;
			if (x >= 0 && x < m_vmap_cano.cols()
				&& y >= 0 && y < m_vmap_cano.rows())
			{
				cudaSafeCall(cudaMemcpy(&canoPos, ((char*)m_vmap_cano.ptr())
					+ m_vmap_cano.step()*y + x * sizeof(float4)
					, sizeof(float3), cudaMemcpyDeviceToHost), "copy cano pos");
				knnIdx = m_warpField->getKnnAt(canoPos);
				knnPtr = &knnIdx;
				canoPosPtr = &canoPos;

				printf("knnIdx[%d]: ", KnnK);
				for (int k = 0; k < KnnK; k++)
					printf("%d\n", knn_k(knnIdx, k));
			}
			lastKnn = knnIdx;
			lastCano = canoPos;
		}
		else
		{
			knnPtr = &lastKnn;
			canoPosPtr = &lastCano;
		}

		// render
		/*m_warpedMesh->renderToImg(userCam, light, img, m_param, m_warpField,
		&m_vmap_curr_pyd[0], &m_vmap_warp, &m_nmap_curr_pyd[0],
		&m_nmap_warp, m_canoMesh, canoPosPtr, knnPtr, &m_kinect_intr, true,
		m_param.view_no_rigid);*/
		float3 volume_size;
		float3 volume_pos;

		volume_pos.x = 0;
		volume_pos.y = 0;
		volume_pos.z = -KINECT_NEAREST_METER - float(m_param.volume_resolution[2] - 1) / m_param.voxels_per_meter;

		volume_size.x = (float)m_param.volume_resolution[0] / m_param.voxels_per_meter;
		volume_size.y = (float)m_param.volume_resolution[1] / m_param.voxels_per_meter;
		volume_size.z = (float)m_param.volume_resolution[2] / m_param.voxels_per_meter;

		m_warpedMesh->renderToImg(userCam, light, img,

			m_param, m_warpField,
			&m_vmap_curr_pyd[0], &m_vmap_warp, &m_nmap_curr_pyd[0],
			&m_nmap_warp, m_canoMesh, canoPosPtr, knnPtr, &m_kinect_intr, true,
			m_param.view_no_rigid);
		//img.create(m_nmap_cano.rows(), m_nmap_cano.cols());
		//generateNormalMap(m_vmap_cano, img);
		
	}

	void WarpProcessor::shadingCanonical(const Camera & userCam, dfusion::LightSource light, dfusion::ColorMap & img, bool use_ray_casting)
	{
		float3 volume_size;
		float3 volume_pos;

		volume_pos.x = 0;
		volume_pos.y = 0;
		volume_pos.z = -KINECT_NEAREST_METER - float(m_param.volume_resolution[2] - 1) / m_param.voxels_per_meter / 2;

		volume_size.x = (float)m_param.volume_resolution[0] / m_param.voxels_per_meter;
		volume_size.y = (float)m_param.volume_resolution[1] / m_param.voxels_per_meter;
		volume_size.z = (float)m_param.volume_resolution[2] / m_param.voxels_per_meter;
		m_canoMesh->renderToImg(userCam, light, img,
			volume_pos,
			volume_size,
			m_param, m_warpField,
			&m_vmap_curr_pyd[0], &m_vmap_warp, &m_nmap_curr_pyd[0],
			&m_nmap_warp, m_canoMesh, nullptr, nullptr, &m_kinect_intr, false, false);
	}

	const WarpField * WarpProcessor::getWarpField() const
	{
		return m_warpField;
	}

	WarpField * WarpProcessor::getWarpField()
	{
		return m_warpField;
	}

	const GpuGaussNewtonSolver * WarpProcessor::getSolver() const
	{
		return m_gsSolver;
	}

	GpuGaussNewtonSolver * WarpProcessor::getSolver()
	{
		return m_gsSolver;
	}

	void WarpProcessor::updateParam(const dfusion::Param & param)
	{
		bool reCreate = false;
		bool reSet = false;

		/*for (int k = 0; k < 3; k++)
		{
			if (m_param.volume_resolution[k] != param.volume_resolution[k])
				reCreate = true;
		}
		if (m_param.voxels_per_meter != param.voxels_per_meter)
			reCreate = true;*/

		m_param = param;
		if (m_warpField)
			m_warpField->setActiveVisualizeNodeId(m_param.view_activeNode_id);

		if (m_canoMesh)
			m_canoMesh->setShowColor(m_param.view_show_color);
		if (m_warpedMesh)
			m_warpedMesh->setShowColor(m_param.view_show_color);

		/*if (reCreate)
			init(param);
		else if (reSet)
			reset();*/
	}

	

	void WarpProcessor::setHandPixel(const std::set<PixelPos>& handpp)
	{
		handPixel = handpp;
	}

	void WarpProcessor::calHandCoveredIdx(std::vector<int>& vertexMap, int & numVertexMaped)
	{
		getWarpedMesh()->toObjMesh(*warpedObjMesh);
		getCanoMesh()->toObjMesh(*canoObjMesh);

		//std::vector<int> vertexMap;
		std::map<int, int> inverseVertexMap;
		ObjMesh nonDupWarped, submeshHand;
		nonDupWarped.cloneFrom(warpedObjMesh);
		numVertexMaped = getVertexMap(nonDupWarped.vertex_list, vertexMap, 0.5*1e-13, &inverseVertexMap);
		removeDuplicateVertexByVertexMap(nonDupWarped, vertexMap, numVertexMaped);
		printf("calHandCoveredIdx:inverseVertexMap size = %d\n", inverseVertexMap.size());
		printf("calHandCoveredIdx:new size = %d\n", numVertexMaped);
		RenderedMesh renderedMesh;
		renderedMesh.init(nonDupWarped, m_kinect_intr, Tbx::Transfo::identity(),
			m_depth_input.rows(), m_depth_input.cols());

		std::set<int> vertexIdxHand;
		renderedMesh.getRenderedHandCoveredIdx(&vertexIdxHand, &handPixel);

		std::vector<int> idxList;
		std::vector<int> subidxList;
		std::set<int>::iterator iter;
		for (iter = vertexIdxHand.begin(); iter != vertexIdxHand.end(); iter++)
		{
			idxList.push_back(*iter);
		}
		nonDupWarped.getSubMesh(idxList, &submeshHand);

		getLargestLinkedSubmesh(submeshHand, subidxList);
		for (int i = 0; i < subidxList.size(); i++)
		{
			subidxList[i] = idxList[subidxList[i]];
		}

		handCoveredVertexIdx.clear();
		for (int i = 0; i < subidxList.size(); i++)
		{
			int index_in_non_dup = subidxList[i];
			int index_in_original_mesh = inverseVertexMap[index_in_non_dup];
			handCoveredVertexIdx.push_back(index_in_original_mesh);
			//printf("i = %d, index_in_original_mesh = %d\n", i, index_in_original_mesh);
		}
		printf("handCoveredVertexIdx size = %d\n", handCoveredVertexIdx.size());
		/*if (vertexMap_to_update)
		*vertexMap_to_update = vertexMap;*/
		/*for (iter = vertexIdxHand.begin(); iter != vertexIdxHand.end(); iter++)
		handCoveredVertexIdx.push_back(*iter);*/
	}

	void WarpProcessor::estimateWarpField()
	{
		if (m_frame_id == 0)
			return;

		// 0. create visibility map of the current warp view
		m_warpedMesh->renderToCanonicalMaps(*m_camera, m_canoMesh, m_vmap_cano, m_nmap_cano);
		m_warpField->warp(m_vmap_cano, m_nmap_cano, m_vmap_warp, m_nmap_warp);

		if (!m_param.fusion_enable_nonRigidSolver)
			return;

#ifdef ENABLE_CPU_DEBUG
		CpuGaussNewton solver;
		solver.init(m_warpField, m_vmap_cano, m_nmap_cano, m_param, m_kinect_intr);
#else
		// icp iteration
		m_gsSolver->init(m_warpField, m_vmap_cano, m_nmap_cano, m_param, m_kinect_intr);
#endif
		float energy = FLT_MAX;
		for (int icp_iter = 0; icp_iter < m_param.fusion_nonRigidICP_maxIter; icp_iter++)
		{
#ifdef ENABLE_CPU_DEBUG
			solver.findCorr(m_vmap_curr_pyd[0], m_nmap_curr_pyd[0], m_vmap_warp, m_nmap_warp);
			solver.solve(false);
#else
			// Gauss-Newton Optimization, findding correspondence internal
			float oldEnergy = energy, data_energy = 0.f, reg_energy = 0.f;
			energy = m_gsSolver->solve(m_vmap_curr_pyd[0], m_nmap_curr_pyd[0],
				m_vmap_warp, m_nmap_warp, &data_energy, &reg_energy);

			//printf("icp, energy(data,reg): %d %f = %f + %f\n", icp_iter, energy, data_energy, reg_energy);
			if (energy > oldEnergy)
				break;

			// update the warp field
			m_gsSolver->updateWarpField();
#endif

			//// update warped mesh and render for visiblity
			//if (icp_iter < m_param.fusion_nonRigidICP_maxIter - 1)
			//{
			//	m_warpField->warp(*m_canoMesh, *m_warpedMesh);
			//	m_warpedMesh->renderToCanonicalMaps(*m_camera, m_canoMesh, m_vmap_cano, m_nmap_cano);
			//}
			m_warpField->warp(m_vmap_cano, m_nmap_cano, m_vmap_warp, m_nmap_warp);
#ifndef ENABLE_CPU_DEBUG
			if (m_param.fusion_post_rigid_factor)
				m_gsSolver->factor_out_rigid();
#endif
		}// end for icp_iter

		 // finally, re-factor out the rigid part across all nodes
	}


	void WarpProcessor::surfaceWarpping()
	{
		//m_marchCube->run(*m_canoMesh);
		m_warpField->warp(*m_canoMesh, *m_warpedMesh);
		m_warpedMesh->renderToDepth(*m_camera, m_depth_prev_pyd[0]);
		createVMap(m_kinect_intr, m_depth_prev_pyd[0], m_vmap_prev_pyd[0]);
		createNMap(m_vmap_prev_pyd[0], m_nmap_prev_pyd[0]);

		for (int i = 1; i < RIGID_ALIGN_PYD_LEVELS; ++i) {
			resizeVMap(m_vmap_prev_pyd[i - 1], m_vmap_prev_pyd[i]);
			resizeNMap(m_nmap_prev_pyd[i - 1], m_nmap_prev_pyd[i]);
		}
	}

	void WarpProcessor::insertNewDeformNodes()
	{
		m_warpField->updateWarpNodes(*m_canoMesh);
	}

	Tbx::Transfo WarpProcessor::rigid_align()
	{
#ifdef ENABLE_BILATERAL_PRE_FILTER
		bilateralFilter(m_depth_input, m_depth_curr_pyd[0]);
#else
		m_depth_input.copyTo(m_depth_curr_pyd[0]);
#endif
		createVMap(m_kinect_intr(0), m_depth_curr_pyd[0], m_vmap_curr_pyd[0]);
		createNMap(m_vmap_curr_pyd[0], m_nmap_curr_pyd[0]);

		//// debug
		//static int a = 0;
		//if (a++ <= 100)
		//{
		//	std::vector<float4> tmp(KINECT_WIDTH*KINECT_HEIGHT);
		//	m_vmap_curr_pyd[0].download(tmp.data(), KINECT_WIDTH*sizeof(float4));
		//	ObjMesh mesh;
		//	for (int i = 0; i < tmp.size(); i++)
		//		mesh.vertex_list.push_back(ldp::Float3(tmp[i].x, tmp[i].y, tmp[i].z));
		//	mesh.saveObj(("D:/" + std::to_string(a) + ".obj").c_str());
		//	system("pause");
		//}
		//// end debug

		//	create pyramid
		for (int i = 1; i < RIGID_ALIGN_PYD_LEVELS; ++i)
			pyrDown(m_depth_curr_pyd[i - 1], m_depth_curr_pyd[i]);

		//	calculate point cloud and normal map
		for (int i = 0; i < RIGID_ALIGN_PYD_LEVELS; ++i)
		{
			//	opengl camera coordinate, -z is camera direction
			createVMap(m_kinect_intr(i), m_depth_curr_pyd[i], m_vmap_curr_pyd[i]);
			createNMap(m_vmap_curr_pyd[i], m_nmap_curr_pyd[i]);
		}

		//	if it is the first frame, no volume to align, so stop here
		if (m_frame_id == 0)
			return Tbx::Transfo().identity();
		else if (!m_param.fusion_enable_rigidSolver)// ldp debug
			return m_warpField->get_rigidTransform();

		// now estimate rigid transform
		Tbx::Transfo c2v = m_warpField->get_rigidTransform().fast_invert();
		Tbx::Mat3	c2v_Rprev = c2v.get_mat3();
		Tbx::Vec3	c2v_tprev = c2v.get_translation();

		Tbx::Mat3	c2v_Rcurr = c2v_Rprev;
		Tbx::Vec3	c2v_tcurr = c2v_tprev;

		const int icp_iterations[] = { m_param.fusion_rigid_ICP_iter[2],
			m_param.fusion_rigid_ICP_iter[1],
			m_param.fusion_rigid_ICP_iter[0] };
		for (int level_index = RIGID_ALIGN_PYD_LEVELS - 1; level_index >= 0; --level_index)
		{
			MapArr& vmap_curr = m_vmap_curr_pyd[level_index];
			MapArr& nmap_curr = m_nmap_curr_pyd[level_index];
			MapArr& vmap_prev = m_vmap_prev_pyd[level_index];
			MapArr& nmap_prev = m_nmap_prev_pyd[level_index];

			int iter_num = icp_iterations[level_index];
			for (int iter = 0; iter < iter_num; ++iter)
			{
				Eigen::Matrix<float, 6, 6, Eigen::RowMajor> A;
				Eigen::Matrix<float, 6, 1> b;

				estimateCombined(convert(c2v_Rcurr), convert(c2v_tcurr), vmap_curr, nmap_curr,
					convert(c2v_Rprev), convert(c2v_tprev), m_kinect_intr(level_index),
					vmap_prev, nmap_prev, m_param.fusion_rigid_distThre, m_param.fusion_rigid_angleThreSin,
					m_rigid_gbuf, m_rigid_sumbuf, A.data(), b.data());

				//checking nullspace
				float det = A.determinant();
				if (fabs(det) < std::numeric_limits<float>::epsilon() || _isnan(det))
				{
					if (_isnan(det))
						std::cout << "qnan" << std::endl;
					else
						std::cout << det << std::endl;
					return c2v.fast_invert();
				}

				Eigen::Matrix<float, 6, 1> result = A.llt().solve(b).cast<float>();

				float alpha = result(0);
				float beta = result(1);
				float gamma = result(2);

				Eigen::Matrix3f Rinc = (Eigen::Matrix3f)Eigen::AngleAxisf(gamma, Eigen::Vector3f::UnitZ()) *
					Eigen::AngleAxisf(beta, Eigen::Vector3f::UnitY()) *
					Eigen::AngleAxisf(alpha, Eigen::Vector3f::UnitX());
				Eigen::Vector3f tinc = result.tail<3>();

				//compose
				c2v_tcurr = convert(Rinc) * c2v_tcurr + convert(tinc);
				c2v_Rcurr = convert(Rinc) * c2v_Rcurr;

			}
		}

		return Tbx::Transfo(c2v_Rcurr, c2v_tcurr).fast_invert();
	}

	Tbx::Transfo WarpProcessor::rigid_align_hand_covered()
	{
		getWarpedMesh()->toObjMesh(*warpedObjMesh);
		return getTransfoByMarker(*warpedObjMesh, *canoObjMesh, handCoveredVertexIdx, false);
	}

	void WarpProcessor::setHandCoveredVertexIdx(const std::vector<int>& vertexIdxList)
	{
		handCoveredVertexIdx = vertexIdxList;
	}

	void WarpProcessor::setHandCoveredVertexIdx(const std::set<int>& vertexIdxSet)
	{
		std::set<int>::iterator iter;
		handCoveredVertexIdx.clear();
		for (iter = vertexIdxSet.begin(); iter != vertexIdxSet.end(); iter++)
		{
			handCoveredVertexIdx.push_back(*iter);
		}
	}

	void WarpProcessor::refineRigidTransform()
	{
		m_warpField->warp(*m_canoMesh, *m_warpedMesh);

		Tbx::Transfo rigid = rigid_align_hand_covered();
		//getchar();
		m_warpField->set_rigidTransform(rigid);
		estimateWarpField();
	}
}

