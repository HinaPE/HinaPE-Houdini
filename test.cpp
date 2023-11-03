#include <CL/sycl.hpp>
constexpr int N = 16;
using namespace sycl;

class IntelGPUSelector : public device_selector {
 public:
  int operator()(const device& Device) const override {
    const std::string DeviceName = Device.get_info<info::device::name>();
    const std::string DeviceVendor = Device.get_info<info::device::vendor>();

    return Device.is_gpu() && (DeviceName.find("Intel") != std::string::npos) ? 100 : 0;
  }
};

int main()
{
  IntelGPUSelector d;
  queue q(d);
  int* data = malloc_shared<int>(N, q);
  q.parallel_for(N, [=](auto i) {
     data[i] = i;
   }).wait();
  for (int i = 0; i < N; i++) std::cout << data[i] << " ";
  free(data, q);
}
