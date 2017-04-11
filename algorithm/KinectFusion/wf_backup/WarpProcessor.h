#pragma once
#include <set>
#include "definations.h"
#include "DynamicFusionParam.h"

class Camera;
class ObjMesh;
namespace dfusion
{
	//#define SPARSE_VOLUME_TESTING
	//class TsdfVolume;
	//class RayCaster;
	class GpuMesh;
	//class MarchingCubes;
	class WarpField;
	class GpuGaussNewtonSolver;
	class SparseVolume;
	class WarpProcessor
	{
	public:
		WarpProcessor();
		~WarpProcessor();

		void init(dfusion::Param param);
		void clear();
		void reset();

		/*void save(const char* volume_name);
		void load(const char* volume_name);*/

		void processFrame(const dfusion::DepthMap& depth, const dfusion::ColorMap& color);

		// if not use_ray_casting, then use marching_cube
		void shading(const Camera& userCam, dfusion::LightSource light, dfusion::ColorMap& img, bool use_ray_casting);
		void shadingCanonical(const Camera& userCam, dfusion::LightSource light, dfusion::ColorMap& img, bool use_ray_casting);
		//void shadingCurrentErrorMap(ColorMap& img, float errorMapRange);

		const WarpField* getWarpField()const;
		WarpField* getWarpField();
		/*const TsdfVolume* getVolume()const;
		TsdfVolume* getVolume();*/
		const GpuGaussNewtonSolver* getSolver()const;
		GpuGaussNewtonSolver* getSolver();
		dfusion::GpuMesh* getWarpedMesh() { return m_warpedMesh; }
		dfusion::GpuMesh* getCanoMesh() { return m_canoMesh; }
		void updateParam(const dfusion::Param& param);

		int getFrameId()const { return m_frame_id; }

		bool hasRawDepth()const { return m_depth_input.rows() > 0; }
		const dfusion::MapArr& getRawDepthNormal()const { return m_nmap_curr_pyd.at(0); }

		// save current canonical mesh and the deformed graph
		//void saveCurrentMeshAndGraph(FILE* pFile);


		
		dfusion::Intr getKinectIntr() { return m_kinect_intr; }
		void setHandPixel(const std::set<PixelPos> & handpp);
		void calHandCoveredIdx(std::vector<int> &vertexMap, int &numVertexMaped);
		const std::vector<int> &getHandVertexIdx() const { return handCoveredVertexIdx; }
	protected:
		void estimateWarpField();
	
		void surfaceWarpping();
		void insertNewDeformNodes();
	

		//void eroseColor(const dfusion::ColorMap& src, dfusion::ColorMap& dst, int nRadius);

		Tbx::Transfo rigid_align();

		Tbx::Transfo rigid_align_hand_covered();
		void setHandCoveredVertexIdx(const std::vector<int> &vertexIdxList);
		void setHandCoveredVertexIdx(const std::set<int> &vertexIdxSet);

		void refineRigidTransform();

		
	protected:
		//===================Sparse Volume Testing===========================

	private:
		dfusion::Param m_param;
		Camera* m_camera;
		/*TsdfVolume* m_volume;
		RayCaster* m_rayCaster;
		MarchingCubes* m_marchCube;*/
		dfusion::GpuMesh* m_canoMesh;
		dfusion::GpuMesh* m_warpedMesh;
		dfusion::WarpField* m_warpField;
		dfusion::Intr m_kinect_intr;

		Tbx::Transfo rigid_transform;

		bool flag_fusion = true;;

		int m_frame_id;

		/** *********************
		* for rigid align
		* **********************/
		enum {
			RIGID_ALIGN_PYD_LEVELS = 3
		};
		dfusion::DepthMap m_depth_input;
		dfusion::ColorMap m_color_input;
		dfusion::ColorMap m_color_tmp;
		std::vector<dfusion::DepthMap> m_depth_curr_pyd;
		std::vector<dfusion::MapArr> m_vmap_curr_pyd;
		std::vector<dfusion::MapArr> m_nmap_curr_pyd;
		std::vector<dfusion::DepthMap> m_depth_prev_pyd;
		std::vector<dfusion::MapArr> m_vmap_prev_pyd;
		std::vector<dfusion::MapArr> m_nmap_prev_pyd;
		DeviceArray2D<float> m_rigid_gbuf;
		DeviceArray<float> m_rigid_sumbuf;
		/** *********************
		* for rigid align with hand-covered
		* **********************/

		std::vector<int> handCoveredVertexIdx;
		ObjMesh* warpedObjMesh;
		ObjMesh* canoObjMesh;
		std::set<PixelPos> handPixel;
		/** *********************
		* for non-rigid align
		* **********************/

		// map of verts in canonical/warped view
		DeviceArray2D<float4> m_vmap_cano;
		DeviceArray2D<float4> m_vmap_warp;
		// map of normals in canonical/warped view
		DeviceArray2D<float4> m_nmap_cano;
		DeviceArray2D<float4> m_nmap_warp;

		GpuGaussNewtonSolver* m_gsSolver;

	};
}