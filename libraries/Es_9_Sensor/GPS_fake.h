#include <array>
#include <tuple>

class GPS_fake
{
public:
    std::tuple<float, float, float> getPosition() const;
    void setPosition(float x, float y, float z);
private:
    std::array<float, 3> position { 0.f };
};