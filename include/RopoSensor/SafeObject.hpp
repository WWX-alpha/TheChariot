#ifndef ROPO_SENSOR_SAFE_OBJECT
#define ROPO_SENSOR_SAFE_OBJECT

#include <cstdint>

namespace RopoSensor {
	template<typename T>
	class SafeObject {
	public:
		SafeObject(T obj) : object(obj), sum(0)
		{
			std::uint8_t *p = (std::uint8_t *)&object;
			for (int i = 0; i < sizeof(T); i++) {
				sum ^= (*p);
				++p;
			}
		}
		bool CheckSafe()
		{
			std::uint8_t result = 0;
			std::uint8_t *p = (std::uint8_t *)&object;
			for (int i = 0; i < sizeof(T); i++) {
				result ^= (*p);
				++p;
			}
			return result == sum;
		}
		T object;
	private:
		std::uint8_t sum;
	};
}

#endif // ROPO_SENSOR_SAFE_OBJECT
