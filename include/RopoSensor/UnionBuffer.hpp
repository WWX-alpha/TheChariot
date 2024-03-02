// UTF - 8
#ifndef ROPO_SENSOR_UNION_BUFFER
#define ROPO_SENSOR_UNION_BUFFER

#include <array>
#include <cstdint>

namespace RopoSensor {
	template<typename T>
	class UnionBuffer {
	public:
		static constexpr std::size_t size = sizeof(T);
		UnionBuffer(T obj) : tempResult{.object = obj}
		{ }
		UnionBuffer(std::array<std::uint8_t, sizeof(T)> buffer) : tempResult{.buffer = buffer}
		{ }
		std::array<std::uint8_t, sizeof(T)> ToBuffer()
		{
			return tempResult.buffer;
		}
		T ToObject()
		{
			return tempResult.object;
		}
	private:
		union {
			T object;
			std::array<std::uint8_t, sizeof(T)> buffer;
		} tempResult;
	};
}

#endif // ROPO_SENSOR_UNION_BUFFER