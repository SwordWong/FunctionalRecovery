#include "KinectFusionProcessor.h"
#include "TsdfVolume.h"
//#include "WarpField.h"
#include "device_utils.h"
#include <helper_math.h>
#include "device_launch_parameters.h"
#include "GpuMesh.h"
namespace dfusion
{
	__device__ __forceinline__ int sign(float a)
	{
		return (a > 0) - (a < 0);
	}



	texture<depthtype, cudaTextureType2D, cudaReadModeElementType> g_depth_tex;
	texture<uchar4, cudaTextureType2D, cudaReadModeNormalizedFloat> g_color_tex;


#if 1
	struct Fusioner
	{
		PtrStepSz<depthtype> depth;

		cudaSurfaceObject_t volumeTex;
		int3 volume_resolution;
		float3 origion;
		float nodeRadius;
		float voxel_size;
		float tranc_dist;
		float max_weight;
		Intr intr;
		float inv_dw_for_fusion2;
		float marchingCube_weightThre;

		Tbx::Mat3 Rv2c;
		Tbx::Point3 tv2c;

		template<int maxK>
		__device__ __forceinline__ void fusion(int x, int y, int z)
		{
			TsdfData rawTsdf = read_tsdf_surface(volumeTex, x, y, z);
			float2 tsdf_weight_prev = unpack_tsdf(rawTsdf);
			float fusion_weight = 0;
			bool suc = true;

			if (!suc)
				return;
			fusion_weight = 1;

			float3 cxyz = convert(Rv2c*(Tbx::Point3(x*voxel_size + origion.x,
				y*voxel_size + origion.y, z*voxel_size + origion.z)) + tv2c);

			float3 uvd = intr.xyz2uvd(cxyz);
			int2 coo = make_int2(__float2int_rn(uvd.x), __float2int_rn(uvd.y));

			if (uvd.x >= 0 && uvd.x < depth.cols && uvd.y >= 0 && uvd.y < depth.rows)
			{
				float depthVal = tex2D(g_depth_tex, coo.x, coo.y)*0.001f;
				float3 dxyz = intr.uvd2xyz(make_float3(coo.x, coo.y, depthVal));
				float sdf = cxyz.z - dxyz.z;

				if (depthVal > KINECT_NEAREST_METER && sdf >= -tranc_dist)
				{
					float tsdf = min(1.0f, sdf / tranc_dist);
					float tsdf_new = (tsdf_weight_prev.x * tsdf_weight_prev.y + fusion_weight * tsdf)
						/ (tsdf_weight_prev.y + fusion_weight);
					float weight_new = min(tsdf_weight_prev.y + fusion_weight, max_weight);
					float4 color = make_float4(0, 0, 0, 0);
					
					write_tsdf_surface(volumeTex, pack_tsdf(tsdf_new, weight_new,
						color), x, y, z);
				}
			}
		}
	};

#endif
	template<int maxK>
	__global__ void tsdf23(Fusioner fs)
	{
		int x = threadIdx.x + blockIdx.x * blockDim.x;
		int y = threadIdx.y + blockIdx.y * blockDim.y;
		int z = threadIdx.z + blockIdx.z * blockDim.z;

		if (x >= fs.volume_resolution.x || y >= fs.volume_resolution.y || z >= fs.volume_resolution.z)
			return;

		fs.fusion<maxK>(x, y, z);
	}// __global__

	void KinectFusionProcessor::fusion()
	{
		dim3 block(32, 8, 2);
		dim3 grid(divUp(m_volume->getResolution().x, block.x),
			divUp(m_volume->getResolution().y, block.y),
			divUp(m_volume->getResolution().z, block.z));

		// bind src to texture
		g_depth_tex.filterMode = cudaFilterModePoint;
		size_t offset;
		cudaChannelFormatDesc desc = cudaCreateChannelDesc<depthtype>();
		cudaBindTexture2D(&offset, &g_depth_tex, m_depth_input.ptr(), &desc,
			m_depth_input.cols(), m_depth_input.rows(), m_depth_input.step());
		assert(offset == 0);
#ifdef ENABLE_COLOR_FUSION
		g_color_tex.filterMode = cudaFilterModePoint;
		desc = cudaCreateChannelDesc<uchar4>();
		cudaBindTexture2D(&offset, &g_color_tex, m_color_input.ptr(), &desc,
			m_color_input.cols(), m_color_input.rows(), m_color_input.step());
		assert(offset == 0);
#endif

		Fusioner fs;
		fs.depth = m_depth_input;
		fs.volumeTex = m_volume->getSurface();
		fs.volume_resolution = m_volume->getResolution();
		fs.origion = m_volume->getOrigion();
		fs.nodeRadius = m_param.warp_radius_search_epsilon;
		fs.voxel_size = m_volume->getVoxelSize();
		fs.tranc_dist = m_volume->getTsdfTruncDist();
		fs.max_weight = m_param.fusion_max_weight;
		fs.intr = m_kinect_intr;
		fs.inv_dw_for_fusion2 = 1.f / (m_param.warp_param_dw_for_fusion*m_param.warp_param_dw_for_fusion);
		fs.marchingCube_weightThre = m_param.marchingCube_min_valied_weight;

		
		Tbx::Transfo tr = rigid_transform;
		fs.Rv2c = tr.get_mat3();
		fs.tv2c = Tbx::Point3(tr.get_translation());

		tsdf23<0> << <grid, block >> >(fs);

		cudaUnbindTexture(&g_depth_tex);
#ifdef ENABLE_COLOR_FUSION
		cudaUnbindTexture(&g_color_tex);
#endif

		cudaSafeCall(cudaGetLastError(), "KinectFusionProcessor::fusion()");
	}

#pragma region --min-filter

	const static int BLOCK_DIM_X = 32;
	const static int BLOCK_DIM_Y = 16;
	const static int MAX_FILTER_RADIUS = 16;
	const static int X_HALO_STEPS = (MAX_FILTER_RADIUS + BLOCK_DIM_X - 1) / BLOCK_DIM_X;
	const static int Y_HALO_STEPS = (MAX_FILTER_RADIUS + BLOCK_DIM_Y - 1) / BLOCK_DIM_Y;
	const static int X_PATCH_PER_BLOCK = 4;
	const static int Y_PATCH_PER_BLOCK = 4;

	template<int Radius>
	__global__ void erose_filter_row(uchar4*  __restrict__ dst,
		const uchar4*  __restrict__ src, int nX, int nY, int pitch)
	{
		// Data cache: threadIdx.x , threadIdx.y
		enum { SMEM_X_LEN = (X_PATCH_PER_BLOCK + 2 * X_HALO_STEPS) * BLOCK_DIM_X };
		enum { SMEM_Y_LEN = BLOCK_DIM_Y };
		__shared__ uchar4 smem[SMEM_Y_LEN][SMEM_X_LEN];

		const int baseX = (blockIdx.x * X_PATCH_PER_BLOCK - X_HALO_STEPS) * BLOCK_DIM_X + threadIdx.x;
		const int baseY = blockIdx.y * BLOCK_DIM_Y + threadIdx.y;
		if (baseY >= nY)
			return;

		src += baseY * pitch + baseX;
		dst += baseY * pitch + baseX;

		//Load main data and right halo
#pragma unroll
		for (int patchId = 0; patchId < X_HALO_STEPS * 2 + X_PATCH_PER_BLOCK; patchId++)
		{
			const int pbx = patchId * BLOCK_DIM_X;
			smem[threadIdx.y][threadIdx.x + pbx] =
				(pbx + baseX < nX && pbx + baseX >= 0) ? src[pbx] : make_uchar4(255, 255, 255, 255);
		}

		//Compute and store results
		__syncthreads();
#pragma unroll
		for (int patchId = X_HALO_STEPS; patchId < X_HALO_STEPS + X_PATCH_PER_BLOCK; patchId++)
		{
			const int pbx = patchId * BLOCK_DIM_X;
			if (baseX + pbx < nX)
			{
				uchar4 s = smem[threadIdx.y][threadIdx.x + pbx];
#pragma unroll
				for (int j = -Radius; j <= Radius; j++)
				{
					if (smem[threadIdx.y][threadIdx.x + pbx + j].x == 0
						&& smem[threadIdx.y][threadIdx.x + pbx + j].y == 0
						&& smem[threadIdx.y][threadIdx.x + pbx + j].z == 0)
						s = make_uchar4(0, 0, 0, 0);
				}
				dst[pbx] = s;
			}
		}
	}

	template<int Radius>
	__global__ void erose_filter_col(uchar4*  __restrict__ dst,
		const uchar4*  __restrict__ src, int nX, int nY, int pitch)
	{
		// Data cache: threadIdx.x , threadIdx.y
		enum { SMEM_X_LEN = BLOCK_DIM_X };
		enum { SMEM_Y_LEN = (Y_PATCH_PER_BLOCK + 2 * Y_HALO_STEPS) * BLOCK_DIM_Y };
		__shared__ uchar4 smem[SMEM_Y_LEN][SMEM_X_LEN];

		const int baseX = blockIdx.x * BLOCK_DIM_X + threadIdx.x;
		const int baseY = (blockIdx.y * Y_PATCH_PER_BLOCK - Y_HALO_STEPS) * BLOCK_DIM_Y + threadIdx.y;
		if (baseX >= nX)
			return;

		src += baseY * pitch + baseX;
		dst += baseY * pitch + baseX;

		//Load main data and lower halo
#pragma unroll
		for (int patchId = 0; patchId < Y_HALO_STEPS * 2 + Y_PATCH_PER_BLOCK; patchId++)
		{
			const int pby = patchId * BLOCK_DIM_Y;
			smem[threadIdx.y + pby][threadIdx.x] =
				(pby + baseY < nY && pby + baseY >= 0) ? src[pby * pitch] : make_uchar4(255, 255, 255, 255);
		}

		//Compute and store results
		__syncthreads();
#pragma unroll
		for (int patchId = Y_HALO_STEPS; patchId < Y_HALO_STEPS + Y_PATCH_PER_BLOCK; patchId++)
		{
			const int pby = patchId * BLOCK_DIM_Y;
			if (baseY + pby < nY)
			{
				uchar4 s = smem[threadIdx.y + pby][threadIdx.x];
#pragma unroll
				for (int j = -Radius; j <= Radius; j++)
				{
					if (smem[threadIdx.y + pby + j][threadIdx.x].x == 0
						&& smem[threadIdx.y + pby + j][threadIdx.x].y == 0
						&& smem[threadIdx.y + pby + j][threadIdx.x].z == 0)
						s = make_uchar4(0, 0, 0, 0);
				}
				dst[pby * pitch] = s;
			}
		}
	}


	template<int Radius>
	static void erose_filter_row_caller(uchar4* dst, const uchar4* src,
		int nX, int nY, int pitch)
	{
		dim3 block(BLOCK_DIM_X, BLOCK_DIM_Y);
		dim3 grid(divUp(nX, block.x*X_PATCH_PER_BLOCK), divUp(nY, block.y), 1);

		erose_filter_row<Radius> << <grid, block >> >(dst, src, nX, nY, pitch);
	}

	template<int Radius>
	static void erose_filter_col_caller(uchar4* dst, const uchar4* src,
		int nX, int nY, int pitch)
	{
		dim3 block(BLOCK_DIM_X, BLOCK_DIM_Y, 1);
		dim3 grid(divUp(nX, block.x), divUp(nY, block.y*Y_PATCH_PER_BLOCK), 1);

		erose_filter_col<Radius> << <grid, block >> >(dst, src, nX, nY, pitch);
	}

	void erose_filter(uchar4* dst_d, const uchar4* src_d,
		int nX, int nY, int pitch,
		int radius, int dim)
	{
		if (src_d == dst_d)
			throw std::exception("min_filter: src and dst cannot be the same memory!");
		if (radius <= 0 || radius >= MAX_FILTER_RADIUS)
			throw std::exception("min_filter: error, non supported kernel size!");
		if (dim > 2 || dim < 0)
			throw std::exception("min_filter: illegal input dim");

		typedef void(*row_caller_t)(uchar4* dst, const uchar4* src,
			int nX, int nY, int pitch);
		typedef void(*col_caller_t)(uchar4* dst, const uchar4* src,
			int nX, int nY, int pitch);
		static const row_caller_t row_callers[MAX_FILTER_RADIUS] =
		{
			0, erose_filter_row_caller<1>, erose_filter_row_caller<2>, erose_filter_row_caller<3>,
			erose_filter_row_caller<4>, erose_filter_row_caller<5>, erose_filter_row_caller<6>,
			erose_filter_row_caller<7>, erose_filter_row_caller<8>, erose_filter_row_caller<9>,
			erose_filter_row_caller<10>, erose_filter_row_caller<11>, erose_filter_row_caller<12>,
			erose_filter_row_caller<13>, erose_filter_row_caller<14>, erose_filter_row_caller<15>,
		};
		static const col_caller_t col_callers[MAX_FILTER_RADIUS] =
		{
			0, erose_filter_col_caller<1>, erose_filter_col_caller<2>, erose_filter_col_caller<3>,
			erose_filter_col_caller<4>, erose_filter_col_caller<5>, erose_filter_col_caller<6>,
			erose_filter_col_caller<7>, erose_filter_col_caller<8>, erose_filter_col_caller<9>,
			erose_filter_col_caller<10>, erose_filter_col_caller<11>, erose_filter_col_caller<12>,
			erose_filter_col_caller<13>, erose_filter_col_caller<14>, erose_filter_col_caller<15>,
		};

		if (dim == 0)
		{
			row_callers[radius](dst_d, src_d, nX, nY, pitch);
		}
		if (dim == 1)
		{
			col_callers[radius](dst_d, src_d, nX, nY, pitch);
		}
		cudaSafeCall(cudaGetLastError(), "erose_filter");
	}
#pragma endregion

	void KinectFusionProcessor::eroseColor(const ColorMap& src, ColorMap& dst, int nRadius)
	{
		m_color_tmp.create(src.rows(), src.cols());
		dst.create(src.rows(), src.cols());

		erose_filter((uchar4*)m_color_tmp.ptr(), (const uchar4*)src.ptr(), src.cols(),
			src.rows(), src.step() / sizeof(uchar4), nRadius, 0);
		erose_filter((uchar4*)dst.ptr(), (const uchar4*)m_color_tmp.ptr(), src.cols(),
			src.rows(), src.step() / sizeof(uchar4), nRadius, 0);
	}
	struct MeshTrans
	{
		const GpuMesh::PointType* vsrc;
		const GpuMesh::PointType* nsrc;
		const GpuMesh::PointType* csrc;

		GpuMesh::PointType* vdst;
		GpuMesh::PointType* ndst;
		GpuMesh::PointType* cdst;
		int num;

		Tbx::Mat3 R;
		float3 t;

		//float3 origion;
		//float invVoxelSize;

		__device__ __forceinline__ void operator()(int tid)
		{
			float3 p = GpuMesh::from_point(vsrc[tid]);
			float3 n = GpuMesh::from_point(nsrc[tid]);

			Tbx::Point3 p_tbx = Tbx::Point3(convert(p));
			Tbx::Vec3 n_tbx = convert(n);
			vdst[tid] = GpuMesh::to_point(convert(R*p_tbx) + t);
			ndst[tid] = GpuMesh::to_point(convert(R*n_tbx));
			cdst[tid] = csrc[tid];
		}
	};

	__global__ void trans_mesh_kernel(MeshTrans mesh_trans)
	{
		unsigned int i = blockIdx.x * (blockDim.x << 3) + threadIdx.x;

#pragma unroll
		for (int k = 0; k < 8; k++)
		{
			if (i < mesh_trans.num)
			{
				mesh_trans(i);
			}
			i += blockDim.x;
		}
	}
	void KinectFusionProcessor::transGpuMesh(GpuMesh& src, GpuMesh& dst, const Tbx::Transfo &tr)
	{
	


		if (src.num() == 0)
			return;

		dst.create(src.num());
		//printf("src size = %d\n", src.num());
		src.lockVertsNormals();
		dst.lockVertsNormals();

		MeshTrans mesh_trans;
		mesh_trans.t = convert(tr.get_translation());
		mesh_trans.R = tr.get_mat3();// Tbx::Quat_cu(m_rigidTransform);
	
		mesh_trans.vsrc = src.verts();
		mesh_trans.nsrc = src.normals();
		mesh_trans.csrc = src.colors();
		mesh_trans.vdst = dst.verts();
		mesh_trans.ndst = dst.normals();
		mesh_trans.cdst = dst.colors();
		mesh_trans.num = src.num();
		

		dim3 block(512);
		dim3 grid(1, 1, 1);
		grid.x = divUp(dst.num(), block.x << 3);
		trans_mesh_kernel << <grid, block >> >(mesh_trans);
		cudaSafeCall(cudaGetLastError(), "trans mesh");

		dst.unlockVertsNormals();
		src.unlockVertsNormals();
	}
}