#ifndef UTILS_HPP
#define UTILS_HPP

inline bool isCollisionPossible(float d1, float d2, float d3) {
    return d1 < 0.4 && d2 < 0.4 && d3 < 0.4;
}

#endif  // !UTILS_HPP
