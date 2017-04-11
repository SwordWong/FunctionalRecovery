#pragma once
#include <set>
#include "definations.h"
#include "DynamicFusionParam.h"
//#include "ObjMesh.h"
//#include "MicrosoftKinect.h"
class ObjMesh;
class Camera;
namespace dfusion
{
	//#define SPARSE_VOLUME_TESTING
	class TsdfVolume;
	class RayCaster;
	class GpuMesh;
	class MarchingCubes;
	class KinectFusionProcessor
	{
	public:
		KinectFusionProcessor();
		~KinectFusionProcessor();

		void init(Param param);
		void clear();
		void reset();

		void save(const char* volume_name);
		void load(const char* volume_name);

		void processFrame(const DepthMap& depth, const ColorMap& color, bool &suc);

		// if not use_ray_casting, then use marching_cube
		void shading(const Camera& userCam, LightSource light, ColorMap& img, bool use_ray_casting);

		const TsdfVolume* getVolume()const;
		TsdfVolume* getVolume();

		
		GpuMesh* getCanoMesh() { return m_canoMesh; }
		void updateParam(const Param& param);

		int getFrameId()const { return m_frame_id; }

		bool hasRawDepth()const { return m_depth_input.rows() > 0; }
		const MapArr& getRawDepthNormal()const { return m_nmap_curr_pyd.at(0); }

		dfusion::Intr getKinectIntr() { return m_kinect_intr; }
	
		void RigidTsdfFusion();
		void surfaceExtractionMC();
		
		const std::vector<MapArr>& getCurrVMAPpyd() { return m_vmap_curr_pyd; }
		const std::vector<MapArr>& getCurrNMAPpyd() { return m_nmap_curr_pyd; }

		Tbx::Transfo getRigidTransform() { return rigid_transform; }

	protected:
		

		void eroseColor(const ColorMap& src, ColorMap& dst, int nRadius);

		Tbx::Transfo rigid_align(bool &suc);

		void transGpuMesh(GpuMesh& src, GpuMesh& dst, const Tbx::Transfo &tr);
		void fusion();
	protected:
		//===================Sparse Volume Testing===========================

	private:
		Param m_param;
		Camera* m_camera;
		TsdfVolume* m_volume;
		RayCaster* m_rayCaster;
		MarchingCubes* m_marchCube;
		GpuMesh* m_canoMesh;

		Intr m_kinect_intr;

		Tbx::Transfo rigid_transform;

		bool flag_fusion = true;;

		int m_frame_id;

		/** *********************
		* for rigid align
		* **********************/
		enum {
			RIGID_ALIGN_PYD_LEVELS = 4
		};
		DepthMap m_depth_input;
		ColorMap m_color_input;
		ColorMap m_color_tmp;
		std::vector<DepthMap> m_depth_curr_pyd;
		std::vector<MapArr> m_vmap_curr_pyd;
		std::vector<MapArr> m_nmap_curr_pyd;
		std::vector<DepthMap> m_depth_prev_pyd;
		std::vector<MapArr> m_vmap_prev_pyd;
		std::vector<MapArr> m_nmap_prev_pyd;
		DeviceArray2D<float> m_rigid_gbuf;
		DeviceArray<float> m_rigid_sumbuf;
		
		//std::set<PixelPos> handPixel;
		/** *********************
		* for non-rigid align
		* **********************/

		// map of verts in canonical/warped view
		DeviceArray2D<float4> m_vmap_cano;
		DeviceArray2D<float4> m_vmap_warp;
		// map of normals in canonical/warped view
		DeviceArray2D<float4> m_nmap_cano;
		DeviceArray2D<float4> m_nmap_warp;


	};
}