#include "KinectFusionProcessor.h"
#include "GpuMesh.h"
#include "RayCaster.h"
#include "MarchingCubes.h"
#include "TsdfVolume.h"
#include "Camera.h"
//#include "WarpField.h"
#include "fmath.h"
#include <eigen\Dense>
#include <eigen\Geometry>
#include "ObjMesh.h"
#include "VolumeData.h"
//#include "CpuGaussNewton.h"
#include "GpuGaussNewtonSolver.h"
#include "SparseVolume.h"
#include "RenderedMesh.h"
#include "meshAlignByMarker.h"
#include "RemoveDuplicateVertex.h"
#include "getLargestLinkedSubmesh.h"
namespace dfusion
{
	//#define ENABLE_CPU_DEBUG
	//#define ENABLE_BILATERAL_PRE_FILTER

	static void check(const char* msg)
	{
		cudaSafeCall(cudaThreadSynchronize());
		printf("%s\n", msg);
	}

#define DFUSION_SAFE_DELETE(buffer)\
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

	KinectFusionProcessor::KinectFusionProcessor()
	{
		m_camera = nullptr;
		m_volume = nullptr;
		m_rayCaster = nullptr;
		m_marchCube = nullptr;
		m_canoMesh = nullptr;
		m_frame_id = 0;
		/*canoObjMesh = new ObjMesh;
		warpedObjMesh = new ObjMesh;*/
#ifdef SPARSE_VOLUME_TESTING
		m_sparseVolume = nullptr;
#endif
	}

	KinectFusionProcessor::~KinectFusionProcessor()
	{
		clear();
		DFUSION_SAFE_DELETE(m_camera);
		DFUSION_SAFE_DELETE(m_volume);
		DFUSION_SAFE_DELETE(m_rayCaster);
		DFUSION_SAFE_DELETE(m_marchCube);
		DFUSION_SAFE_DELETE(m_canoMesh);
	}

	void KinectFusionProcessor::init(Param param)
	{
		clear();
		//glutInit();
		m_param = param;
		// allocation----------------------

		// camera
		if (m_camera == nullptr)
			m_camera = new Camera();
		m_camera->setViewPort(0, KINECT_WIDTH, 0, KINECT_HEIGHT);
		/*m_camera->setPerspective(KINECT_DEPTH_V_FOV, float(KINECT_WIDTH) / KINECT_HEIGHT,
		KINECT_NEAREST_METER, 30.f);*/
		m_camera->setPerspective(KINECT_DEPTH_V_FOV, float(KINECT_WIDTH) / KINECT_HEIGHT,
			0.1, 30.f);
		const float l = m_camera->getViewPortLeft();
		const float r = m_camera->getViewPortRight();
		const float t = m_camera->getViewPortTop();
		const float b = m_camera->getViewPortBottom();
		const float	f = (b - t) * 0.5f / tanf(m_camera->getFov() * fmath::DEG_TO_RAD * 0.5f);
		m_kinect_intr = Intr(f, f, (l + r) / 2, (t + b) / 2);

		// volume
		if (m_volume == nullptr)
			m_volume = new TsdfVolume();
		m_volume->init(make_int3(
			m_param.volume_resolution[0],
			m_param.volume_resolution[1],
			m_param.volume_resolution[2]),
			1.f / m_param.voxels_per_meter,
			make_float3(-(m_param.volume_resolution[0] - 1)*0.5f / m_param.voxels_per_meter,
				-(m_param.volume_resolution[1] - 1) * 0.5f / m_param.voxels_per_meter,
				-KINECT_NEAREST_METER - float(m_param.volume_resolution[2] - 1) / m_param.voxels_per_meter)
		);
#ifdef SPARSE_VOLUME_TESTING
		if (m_sparseVolume == nullptr)
			m_sparseVolume = new SparseVolume();
		m_sparseVolume->init(make_int3(
			m_param.volume_resolution[0],
			m_param.volume_resolution[1],
			m_param.volume_resolution[2]),
			1.f / m_param.voxels_per_meter,
			make_float3(-(m_param.volume_resolution[0] - 1)*0.5f / m_param.voxels_per_meter,
				-(m_param.volume_resolution[1] - 1) * 0.5f / m_param.voxels_per_meter,
				-KINECT_NEAREST_METER - float(m_param.volume_resolution[2] - 1) / m_param.voxels_per_meter)
		);
#endif

		// mesh
		if (m_canoMesh == nullptr)
			m_canoMesh = new GpuMesh();

		m_canoMesh->setShowColor(m_param.view_show_color);

		// marching cube
		if (m_marchCube == nullptr)
			m_marchCube = new MarchingCubes();
		m_marchCube->init(m_volume, m_param);

		// ray casting
		if (m_rayCaster == nullptr)
			m_rayCaster = new RayCaster();
		m_rayCaster->init(*m_volume);


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

	void KinectFusionProcessor::clear()
	{
		if (m_camera)
			m_camera->reset();
		if (m_volume)
			m_volume->reset();
		if (m_rayCaster)
			m_rayCaster->clear();
		if (m_canoMesh)
			m_canoMesh->release();
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

	void KinectFusionProcessor::updateParam(const Param& param)
	{
		bool reCreate = false;
		bool reSet = false;

		for (int k = 0; k < 3; k++)
		{
			if (m_param.volume_resolution[k] != param.volume_resolution[k])
				reCreate = true;
		}
		if (m_param.voxels_per_meter != param.voxels_per_meter)
			reCreate = true;

		m_param = param;

		if (m_canoMesh)
			m_canoMesh->setShowColor(m_param.view_show_color);
	
		if (reCreate)
			init(param);
		else if (reSet)
			reset();
	}

	void KinectFusionProcessor::reset()
	{
		// camera
		m_camera->setModelViewMatrix(ldp::Mat4f().eye());

		// volume
		m_volume->reset();

		// mesh
		m_canoMesh->release();
		//m_warpedMesh->release();

		m_frame_id = 0;
	}

	void KinectFusionProcessor::processFrame(const DepthMap& depth,
		const ColorMap& color,
		bool &suc)
	{
		depth.copyTo(m_depth_input);

		color.copyTo(m_color_input);
		//bool suc;
		Tbx::Transfo rigid = rigid_align(suc);
		rigid_transform = rigid;
		if (m_frame_id == 0 || suc)
		{
			
			RigidTsdfFusion();
		}
		surfaceExtractionMC();
		if(suc)
		m_frame_id++;
	}

	void KinectFusionProcessor::shading(const Camera& userCam, LightSource light,
		ColorMap& img, bool use_ray_casting)
	{
		if (use_ray_casting)
		{
			Camera cam = *m_camera;
			cam.setModelViewMatrix(userCam.getModelViewMatrix()*m_camera->getModelViewMatrix());
			m_rayCaster->setCamera(cam);
			m_rayCaster->shading(light, img);
		}
		else
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
			
			float3 volume_size;
			float3 volume_pos;

			volume_pos.x = 0;
			volume_pos.y = 0;
			volume_pos.z = -KINECT_NEAREST_METER - float(m_param.volume_resolution[2] - 1) / m_param.voxels_per_meter;

			volume_size.x = (float)m_param.volume_resolution[0] / m_param.voxels_per_meter;
			volume_size.y = (float)m_param.volume_resolution[1] / m_param.voxels_per_meter;
			volume_size.z = (float)m_param.volume_resolution[2] / m_param.voxels_per_meter;

			m_canoMesh->renderToImgOnlyMesh(userCam, light, img,
				m_param, 
				&m_vmap_curr_pyd[0], &m_vmap_warp, &m_nmap_curr_pyd[0],
				&m_nmap_warp, m_canoMesh, canoPosPtr, knnPtr, &m_kinect_intr, &rigid_transform,
				&volume_pos, &volume_size);
		}
	}

	

	const TsdfVolume* KinectFusionProcessor::getVolume()const
	{
		return m_volume;
	}

	TsdfVolume* KinectFusionProcessor::getVolume()
	{
		return m_volume;
	}

	

	void KinectFusionProcessor::save(const char* volume_name)
	{
		m_volume->save(volume_name);

		std::string path, purename, ext;
		ldp::fileparts(volume_name, path, purename, ext);
		std::string name1 = fullfile(path, purename + ".warpfield");
		//m_warpField->save(name1.c_str());
		std::string name2 = fullfile(path, purename + ".param.txt");
		m_param.save(name2.c_str());
	}

	void KinectFusionProcessor::load(const char* volume_name)
	{
		int3 old_res = m_volume->getResolution();
		m_volume->load(volume_name);
		int3 new_res = m_volume->getResolution();
		if (old_res.x != new_res.x || old_res.y != new_res.y || old_res.z != new_res.z)
			throw std::exception("error in loading volumes, size not matched with pre-allocated!");

		std::string path, purename, ext;
		ldp::fileparts(volume_name, path, purename, ext);
		std::string name1 = fullfile(path, purename + ".warpfield");
		//m_warpField->load(name1.c_str());

		surfaceExtractionMC();
		//m_warpedMesh->renderToCanonicalMaps(*m_camera, m_canoMesh, m_vmap_cano, m_nmap_cano);
		//m_warpField->warp(m_vmap_cano, m_nmap_cano, m_vmap_warp, m_nmap_warp);
		m_frame_id = 1; // !=0 to prevent trivial case.
	}

	

	void KinectFusionProcessor::RigidTsdfFusion()
	{
		fusion();

#ifdef SPARSE_VOLUME_TESTING
		cudaSafeCall(cudaThreadSynchronize(), "1");
		ldp::tic();
		VoxelBlockAllocation(m_depth_input);
		cudaSafeCall(cudaThreadSynchronize(), "2");
		VisibleVoxelBlockSelection();
		cudaSafeCall(cudaThreadSynchronize(), "3");
		printf("%d %d %d %d\n", m_sparseVolume->voxel_block_number_,
			m_sparseVolume->selected_voxel_block_number_,
			m_sparseVolume->hash_entry_size_,
			m_sparseVolume->selected_hash_entry_number_);
		VoxelBlockUpdate(m_depth_input);
		cudaSafeCall(cudaThreadSynchronize(), "4");
		ldp::toc("sparse fusion");
#endif
	}

	void KinectFusionProcessor::surfaceExtractionMC()
	{
		
		ObjMesh tmpmesh;
		char filename[100];
		m_marchCube->run(*m_canoMesh);
		
		//transGpuMesh(*m_canoMesh, *m_warpedMesh, rigid_transform);
		
		/*m_warpedMesh->toObjMesh(tmpmesh);
		sprintf(filename, "af%4d.obj", m_frame_id);
		tmpmesh.saveObj(filename);*/
		/*m_warpedMesh->toObjMesh(tmpmesh);
		tmpmesh.saveObj("ww.obj");*/
		//m_warpedMesh->renderToDepth(*m_camera, m_depth_prev_pyd[0]);
		m_canoMesh->renderToDepth(*m_camera, m_depth_prev_pyd[0], &rigid_transform.fast_invert());
		createVMap(m_kinect_intr, m_depth_prev_pyd[0], m_vmap_prev_pyd[0]);
		createNMap(m_vmap_prev_pyd[0], m_nmap_prev_pyd[0]);

		for (int i = 1; i < RIGID_ALIGN_PYD_LEVELS; ++i) {
			resizeVMap(m_vmap_prev_pyd[i - 1], m_vmap_prev_pyd[i]);
			resizeNMap(m_nmap_prev_pyd[i - 1], m_nmap_prev_pyd[i]);
		}
		//exit(1);
	}

	

	Tbx::Transfo KinectFusionProcessor::rigid_align(bool &suc)
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
		//else if (!m_param.fusion_enable_rigidSolver)// ldp debug
		//	return m_warpField->get_rigidTransform();

		// now estimate rigid transform
		//Tbx::Transfo c2v = m_warpField->get_rigidTransform().fast_invert();
		Tbx::Transfo c2v = rigid_transform.fast_invert();
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
					suc = false;
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
		suc = true;
		return Tbx::Transfo(c2v_Rcurr, c2v_tcurr).fast_invert();
	}



}