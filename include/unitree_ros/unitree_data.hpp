#ifndef UNITREE_DATA_HPP
#define UNITREE_DATA_HPP

/**
 * @brief Enum containing the possible robot's mode
 *
 * @param MODE_IDDLE
 * @param WALK_W_VEL
 * @param WALK_W_POS
 * @param STAND_DOWN
 * @param STAND_UP
 * @param DAMPING_MODE
 */
typedef enum {
    MODE_IDDLE = 0,
    WALK_W_VEL = 2,
    WALK_W_POS = 3,
    STAND_DOWN = 5,
    STAND_UP = 6,
    DAMPING_MODE = 7,
} mode_enum;

/**
 * @brief Enum containing the possible robot's gait types
 *
 *  @param GAITYPE_IDDLE
 *  @param TROT
 *  @param TROT_RUNNING
 *  @param CLIMB_STAIR
 *  @param TROT_OBSTACLE
 */
typedef enum {
    GAITYPE_IDDLE = 0,
    TROT = 1,
    TROT_RUNNING = 2,
    CLIMB_STAIR = 3,
    TROT_OBSTACLE = 4,
} gaitype_enum;

/**
 * @brief Enum containing the possible robot speed levels
 */
typedef enum {
    LOW_SPEED = 0,
    MEDIUM_SPEED = 1,
    HIGH_SPEED = 2,
} speed_level_enum;

/**
 * @brief Enum containing the possible robot status, for LEDs
 */
typedef enum {
    READY,
    MOVING,
    IDDLE,
    BATTERY_LOW,
    ERROR,
} robot_status_e;

/**
 * @brief Position data type
 *
 * @param x
 * @param y
 * @param z
 */
typedef struct {
    float x;
    float y;
    float z;
} position_t;

/**
 * @brief Orientation data type
 *
 * @param x
 * @param y
 * @param z
 * @param w
 */
typedef struct {
    float x;
    float y;
    float z;
    float w;
} orientation_t;

/**
 * @brief Pose data type
 *
 * @param position
 * @param orientation
 */
typedef struct {
    position_t position;
    orientation_t orientation;
} pose_t;

/**
 * @brief Velocity data type
 *
 * @param position
 * @param orientation
 */
typedef struct {
    float x;
    float y;
    float yaw;
} velocity_t;

/**
 * @brief Odometry data type
 *
 * @param position
 * @param orientation
 */
typedef struct {
    pose_t pose;
    velocity_t velocity;
} odom_t;

/**
 * @brief Radar ranges data type
 *
 * @param left
 * @param front
 * @param right
 */
typedef struct {
    float left;
    float front;
    float right;
} sensor_ranges_t;

#endif  // !#ifndef UNITREE_DATA_HPP
