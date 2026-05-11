namespace SGPowerControl
{
    class PowerUpData_t
    {
    private:
        IMotorInterface* motor_interface_;
        
    public:
        Math::RLS<2> rls;

        Matrixf<2, 1> samples;
        Matrixf<2, 1> params;

        float MAXPower;
        PowerUpData_t() : rls(1e-5f, 0.99999f), motor_interface_(nullptr), Init_flag(false)
        {
            // 初始化成员变量
            k1 = k2 = k3 = k0 = 0.0f;
            Energy = 0.0f;
            EstimatedPower = 0.0f;
            Cur_EstimatedPower = 0.0f;
            EffectivePower = 0.0f;
            E_lower = 0.0f;
            E_upper = 0.0f;
        }

        ~PowerUpData_t() {
            if (motor_interface_) {
                delete motor_interface_;
            }
        }

        bool is_RLS = false;

        /* data */
        float k1, k2, k3, k0;

        float Energy;

        float EstimatedPower;
        float Cur_EstimatedPower;

        float Initial_Est_power[4];

        float EffectivePower;

        float pMaxPower[4];
        double Cmd_MaxT[4];
		
		bool Init_flag;

        // 定义阈值参数
        float E_lower;
        float E_upper;

        // 设置电机接口
        void SetMotorInterface(IMotorInterface* interface) {
            delete motor_interface_;
            motor_interface_ = interface;
        }

        // 统一的功率计算方法
        void UpRLS(PID *pid, const float toque_const, const float rpm_to_rads);
        
        // 等比缩放的最大分配功率
        void UpScaleMaxPow(PID *pid);
        
        // 计算应分配的力矩
        void UpCalcMaxTorque(float *final_Out, PID *pid, const float toque_const, const float rpm_to_rads);

        // 能量环相关成员
        float energy_setpoint;           // 能量目标值
        float energy_feedback;           // 能量反馈值（电容能量）
        float energy_error;              // 能量误差
        float energy_derivative;         // 能量变化率
        float energy_integral;           // 能量积分
        float energy_pd_output;          // PD控制器输出
        float energy_max_power;          // 功率上限值
        float energy_min_power;          // 功率下限值
        
        // 能量环PD控制器参数
        float kp_energy;                 // 比例增益
        float kd_energy;                 // 微分增益
        
        // 能量环初始化
        void InitEnergyRing(float setpoint, float kp, float kd);
        
        // 能量环更新
        void UpdateEnergyRing(float feedback, float dt);
        
        // 获取功率上限值
        float GetMaxPower() const { return energy_max_power; }
        
        // 获取功率下限值
        float GetMinPower() const { return energy_min_power; }
    };

    class PowerTask_t
    {
    public:
        PowerTask_t()
        {
            SetDefaultConfig();
        }
        void SetDefaultConfig() {
            // 轮向电机配置 (DJI 3508)
            Wheel_PowerData.MAXPower     = 60.0f;
            Wheel_PowerData.k1           = 1.08900523f;
            Wheel_PowerData.k2           = 0.814881027f;
            Wheel_PowerData.k3           = 5.0f;
            Wheel_PowerData.is_RLS       = true;
            Wheel_PowerData.E_upper      = 1000.0f;
            Wheel_PowerData.E_lower      = 500.0f;

            // 舵向电机配置 (LK 4005)
            String_PowerData.MAXPower    = 60.0f * 0.6f;
            String_PowerData.k1          = 0.182967603f;
            String_PowerData.k2          = 8.78055f;
            String_PowerData.k3          = 5.0f;
            String_PowerData.is_RLS      = false;
            String_PowerData.E_upper     = 500.0f;
            String_PowerData.E_lower     = 100.0f;
            
            // 能量环默认参数
            Wheel_PowerData.energy_setpoint = 750.0f;  // 默认能量目标值
            Wheel_PowerData.kp_energy = 0.1f;          // 默认比例增益
            Wheel_PowerData.kd_energy = 0.01f;         // 默认微分增益
            Wheel_PowerData.energy_min_power = 15.0f;  // 最小功率限制
        }

        // 初始化电机接口
        void InitMotorInterfaces(BSP::Motor::Dji::GM3508<4>& wheel_motor, BSP::Motor::LK::LK4005<4>& string_motor) {
            Wheel_PowerData.SetMotorInterface(new DjiMotorAdapter(wheel_motor));
            String_PowerData.SetMotorInterface(new LkMotorAdapter(string_motor));
            
            // 初始化能量环
            Wheel_PowerData.InitEnergyRing(Wheel_PowerData.energy_setpoint, Wheel_PowerData.kp_energy, Wheel_PowerData.kd_energy);
            String_PowerData.InitEnergyRing(String_PowerData.energy_setpoint, String_PowerData.kp_energy, String_PowerData.kd_energy);
        }

        PowerUpData_t String_PowerData;
        PowerUpData_t Wheel_PowerData;

        inline float GetEstWheelPow() const
        {
            return Wheel_PowerData.EstimatedPower;
        }

        inline float GetEstStringPow() const
        {
            return String_PowerData.EstimatedPower;
        }

        inline void setMaxPower(float maxPower)
        {
            Wheel_PowerData.MAXPower  = maxPower;
            String_PowerData.MAXPower = maxPower * 0.6f;
        }

        inline uint16_t getMAXPower() const
        {
            return static_cast<uint16_t>(Wheel_PowerData.MAXPower);
        }
        

    };
} // namespace SGPowerControl