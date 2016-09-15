
#include "utility.hpp"


 void utility::getRainbowColor(float value, float& r, float& g, float& b){
    // this is HSV color palette with hue values going only from 0.0 to 0.833333.
    value = std::min(value, 1.0f);
    value = std::max(value, 0.0f);

    float color[3];

    float h = value * 5.0f + 1.0f;
    int i = floor(h);
    float f = h - i;
    if (!(i & 1))
    f = 1 - f; // if i is even
    float n = 1 - f;

    if (i <= 1)
    color[0] = n, color[1] = 0, color[2] = 1;
    else if (i == 2)
    color[0] = 0, color[1] = n, color[2] = 1;
    else if (i == 3)
    color[0] = 0, color[1] = 1, color[2] = n;
    else if (i == 4)
    color[0] = n, color[1] = 1, color[2] = 0;
    else if (i >= 5)
    color[0] = 1, color[1] = n, color[2] = 0;

    r = color[0];
    g = color[1];
    b = color[2];
  }


