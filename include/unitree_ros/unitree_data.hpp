#ifndef UNITREE_DATA_HPP
#define UNITREE_DATA_HPP

typedef enum {
    MODE_IDDLE = 0,
    WALK_W_VEL = 1,
    WALK_W_POS = 2,
    STAND_DOWN = 5,
    STAND_UP = 6,
    DAMPING_MODE = 7,
} mode_enum;

typedef enum {
    GAITYPE_IDDLE = 0,
    TROT = 1,
    TROT_RUNNING = 2,
    CLIMB_STAIR = 3,
    TROT_OBSTACLE = 4,
} gaitype_enum;

typedef struct {
    float x;
    float y;
    float z;
} position_t;

typedef struct {
    float x;
    float y;
    float z;
    float w;
} orientation_t;

typedef struct {
    position_t position;
    orientation_t orientation;
} pose_t;

typedef struct {
    float x;
    float y;
    float yaw;
} velocity_t;

typedef struct {
    pose_t pose;
    velocity_t velocity;
} odom_t;

typedef struct {
    float left;
    float front;
    float right;
    float bottom;
} sensor_ranges_t;

#endif  // !#ifndef UNITREE_DATA_HPP
