#ifndef _DIST_KERNEL_CU_
#define _DIST_KERNEL_CU_

#include <helper_cuda.h>
#include <helper_math.h>

__device__ uint rgbaFloatToInt(float4 rgba) {
  rgba.x = __saturatef(rgba.x);  // clamp to [0.0, 1.0]
  rgba.y = __saturatef(rgba.y);
  rgba.z = __saturatef(rgba.z);
  rgba.w = __saturatef(rgba.w);
  return (uint(rgba.w * 255) << 24) | (uint(rgba.z * 255) << 16) |
         (uint(rgba.y * 255) << 8) | uint(rgba.x * 255);
}

__global__ void d_render(uint *d_output, uint imageW, uint imageH, float scale,
                         float2 view_center) {
  uint x = blockIdx.x * blockDim.x + threadIdx.x;
  uint y = blockIdx.y * blockDim.y + threadIdx.y;

  if ((x >= imageW) || (y >= imageH)) {
    return;
  }

  // These int-float cast operations aren't cheap
  float u = ((x / (float)imageW) * 2.0f - 1.0f);  // + view_center.x;
  float v = ((y / (float)imageH) * 2.0f - 1.0f);  // + view_center.y;

  float4 sum = make_float4(0.0f);
  float distance = hypot(u, v);
  sum.x = distance;
  sum.y = distance / sqrt(distance) * v;
  sum.z = distance * distance;
  sum.w = 1.0;

  d_output[y * imageW + x] = rgbaFloatToInt(sum);
}

extern "C" void render_kernel(dim3 gridSize, dim3 blockSize, uint *d_output,
                              uint imageW, uint imageH, float scale,
                              float2 view_center) {
  printf("%9.2f, %9.2f: %9.2f \n", view_center.x, view_center.y, scale);
  d_render<<<gridSize, blockSize>>>(d_output, imageW, imageH, scale,
                                    view_center);
}

#endif  // #ifndef _DIST_KERNEL_CU_
