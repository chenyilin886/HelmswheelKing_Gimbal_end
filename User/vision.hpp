#ifndef VISION_HPP
#define VISION_HPP

#pragma once

#include <cstdint>
#include <cstring>

namespace Comm
{

class Vision
{
public:
    static constexpr uint8_t TX_FRAME_HEAD1 = 0x39;
    static constexpr uint8_t TX_FRAME_HEAD2 = 0x39;
    static constexpr uint8_t TX_FRAME_TAIL = 0xFF;
    static constexpr uint8_t TX_FRAME_SIZE = 29;

    static constexpr uint8_t RX_FRAME_HEAD1 = 0x39;
    static constexpr uint8_t RX_FRAME_HEAD2 = 0x39;
    static constexpr uint8_t RX_FRAME_SIZE = 19;

    struct TxGimbal
    {
        float quat_w;
        float quat_x;
        float quat_y;
        float quat_z;
        uint32_t timestamp;
    };

    struct TxOther
    {
        float bullet_speed;
        uint8_t enemy_color;
        uint8_t vision_mode;
        uint8_t tail;
    };

    struct RxTarget
    {
        float pitch_angle;
        float yaw_angle;
        uint32_t timestamp;
    };

    struct RxOther
    {
        uint8_t vision_ready;
        uint8_t fire;
        uint8_t tail;
        uint8_t aim_x;
        uint8_t aim_y;
    };

    struct DebugData
    {
        uint32_t tx_count;
        uint32_t rx_count;
        
        float tx_quat_w;
        float tx_quat_x;
        float tx_quat_y;
        float tx_quat_z;
        float tx_bullet_speed;
        uint8_t tx_enemy_color;
        uint8_t tx_vision_mode;
        uint32_t tx_timestamp;
        
        float rx_pitch_angle;
        float rx_yaw_angle;
        uint8_t rx_vision_ready;
        uint8_t rx_fire;
        uint8_t rx_aim_x;
        uint8_t rx_aim_y;
        uint32_t rx_timestamp;
        
        bool vision_flag;
        bool rx_frame_valid;
        uint8_t tx_buffer[32];
        uint8_t rx_buffer[19];
    };

    Vision() : tx_timestamp_(0), vision_flag_(false), fire_update_count_(0) {}

    void send(float quat_w, float quat_x, float quat_y, float quat_z);
    void receive();

    float getTargetYaw() const { return target_yaw_; }
    float getTargetPitch() const { return target_pitch_; }
    bool getVisionFlag() const { return vision_flag_; }
    uint8_t getFire() const { return rx_other_.fire; }
    uint32_t getFireUpdateCount() const { return fire_update_count_; }
    uint8_t getVisionReady() const { return rx_other_.vision_ready; }
    uint8_t getAimX() const { return rx_other_.aim_x; }
    uint8_t getAimY() const { return rx_other_.aim_y; }

    void setVisionMode(uint8_t mode) { tx_other_.vision_mode = mode; }
    void setEnemyColor(uint8_t color) { tx_other_.enemy_color = color; }
    void setBulletSpeed(float speed) { tx_other_.bullet_speed = speed; }

    const uint8_t *getTxBuffer() const { return tx_buffer_; }
    uint8_t *getRxBuffer() { return rx_buffer_; }
    static constexpr uint8_t getTxSize() { return TX_FRAME_SIZE; }
    static constexpr uint8_t getRxSize() { return RX_FRAME_SIZE; }
    
    const DebugData& getDebugData() const { return debug_data_; }
    DebugData& getDebugData() { return debug_data_; }

private:
    void packTxFrame();
    bool parseRxFrame();

    TxGimbal tx_gimbal_{};
    TxOther tx_other_{};
    RxTarget rx_target_{};
    RxOther rx_other_{};

    uint8_t tx_buffer_[TX_FRAME_SIZE]{};
    uint8_t rx_buffer_[RX_FRAME_SIZE]{};

    uint32_t tx_timestamp_;
    float target_yaw_{0.0f};
    float target_pitch_{0.0f};
    bool vision_flag_;
    uint32_t fire_update_count_;
    uint8_t last_fire_value_{0};
    bool fire_initialized_{false};
    
    DebugData debug_data_{};
};

extern Vision vision;

}

#endif
