#include "force_cmd.hpp"
#include "gravcomp.hpp"
#include <cmath>

using namespace std;
const double PI = 3.141592653589793;

namespace force_cmd
{
    // ForceKeep指令模拟阻抗控制
    // 结构体内储存数据不随循环重新初始化
	struct ForceKeep::Imp {
			//Flag
			bool init = false;
			bool contact_check = false;
			//Force Compensation Parameter
			double comp_f[6]{ 0 };
			//Arm1
			double arm1_init_force[6]{ 0 };
			double arm1_p_vector[6]{ 0 };
			double arm1_l_vector[6]{ 0 };
			//Arm2
			double arm2_init_force[6]{ 0 };
			double arm2_p_vector[6]{ 0 };
			double arm2_l_vector[6]{ 0 };
			//Desired Pos, Vel, Acc, Foc
			double arm1_x_d[6]{ 0 };
			double arm2_x_d[6]{ 0 };
			double v_d[6]{ 0 };
			double a_d[6]{ 0 };
			double f_d[6]{ 0 };
			//Current Vel
			double v_c[6]{ 0 };
			//Impedence Parameter
			double a1_K[6]{ 100,100,100,5,5,5 };
			double a1_B[6]{ 100,100,100,5,5,5 };
			double a1_M[6]{ 1,1,1,2,2,2 };
			double a2_K[6]{ 100,100,100,15,15,15 };
			double a2_B[6]{ 100,100,100,15,15,15 };
			double a2_M[6]{ 1,1,1,10,10,10 };
			double Ke[6]{ 220000,220000,220000,220000,220000,220000 };
			//Counter
			int contact_count = 0;
			//Test
			double actual_force[6]{ 0 };
			//Switch Model
			int m_;
			//Force Buffer
			std::array<double, 10> force_buffer[6] = {};
			int buffer_index[6]{ 0 };
		};
	// PrepareNrt函数在非实时线程中执行，通常用于执行初始设置，例如设置电机选项等
    auto ForceKeep::prepareNrt() -> void
	{
        // 设置电机选项，禁用位置二阶连续性检查，取消所有检查（check_none）
        // 注意！！取消所有检查会才能在仿真中运行，实机控制不能取消所有检查！！！会导致忽略必要的使能，电流过载检查
		for (auto& m : motorOptions()) m =
			aris::plan::Plan::CHECK_NONE |
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
		GravComp gc;
        // 实机运行时此处需要加载重力补偿向量
		// gc.loadPLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);
		mout() << "Load P & L Vector" << std::endl;
	}
	auto ForceKeep::executeRT() -> int
	{
		//dual transform modelbase into multimodel
		auto& dualArm = dynamic_cast<aris::dynamic::MultiModel&>(modelBase()[0]);
		//at(0) -> Arm1 -> white
		auto& arm1 = dualArm.subModels().at(0);
		//at(1) -> Arm2 -> blue
		auto& arm2 = dualArm.subModels().at(1);
		//transform to model
		auto& model_a1 = dynamic_cast<aris::dynamic::Model&>(arm1);
		auto& model_a2 = dynamic_cast<aris::dynamic::Model&>(arm2);
		//End Effector
		auto& eeA1 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a1.generalMotionPool().at(0));
		auto& eeA2 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a2.generalMotionPool().at(0));
		static double tolerance = 0.0001;
		static double init_angle[12] =
		{ 0, 0, 5 * PI / 6, -5 * PI / 6, -PI / 2, 0 ,
		0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, 0 };
		static double max_vel[6]{ 0.2,0.2,0.2,0.0005,0.0005,0.0001 };
		static double trigger_force[6]{ 0.5,0.5,0.5,0.001,0.001,0.001 };
		static double max_force[6]{ 10,10,10,5,5,5 };
		static double trigger_vel[6]{ 0.0001,0.0001,0.0001,0.0001,0.0001,0.0001 };

		GravComp gc;
		double current_angle[12]{ 0 };
		double current_sa_angle[6]{ 0 };
		double comp_force[6]{ 0 };
		double current_pm[16]{ 0 };
		double current_pos[6]{ 0 };
		double current_force[6]{ 0 };
		double actual_force[6]{ 0 };
		double filtered_force[6]{ 0 };
		double transform_force[6]{ 0 };

		imp_->m_ = int32Param("model");
        // 所有lambda函数求解的位置都为下一个count的位置，仅为了方便控制

        // 读取力传感器原始数据，ethercat序号需要根据实际情况调整，仿真中无法使用
        // 当init为false时，读取原始力数据，默认为初始需要减去的偏置
        // 当init为true时，输出减掉初始偏置后的数据
		auto getForceData = [&](double* data_, int m_, bool init_)
		{
			int raw_force[6]{ 0 };
			for (std::size_t i = 0; i < 6; ++i)
			{
				if (ecMaster()->slavePool()[9 + 9 * m_].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
					mout() << "error" << std::endl;

				data_[i] = (static_cast<double>(raw_force[i]) / 1000.0);

			}
			if (!init_)
			{
				mout() << "Compensate Init Force" << std::endl;
			}
			else
			{
				if (m_ == 0)
				{
					for (std::size_t i = 0; i < 6; ++i)
					{
						data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm1_init_force[i];
					}
				}
				else if (m_ == 1)
				{
					for (std::size_t i = 0; i < 6; ++i)
					{
						data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm2_init_force[i];
					}
				}
				else
				{
					mout() << "Wrong Model" << std::endl;
				}
			}
		};

        // 直接控制双臂电机目标位置，实机控制时避免速度超限，不会求解运动学
		auto daJointMove = [&](double target_mp_[12])
		{
			double current_angle[12] = { 0 };
			double move = 0.00005;
			for (int i = 0; i < 12; i++)
			{
				current_angle[i] = controller()->motorPool()[i].targetPos();
			}

			for (int i = 0; i < 12; i++)
			{
				if (current_angle[i] <= target_mp_[i] - move)
				{
					controller()->motorPool()[i].setTargetPos(current_angle[i] + move);
				}
				else if (current_angle[i] >= target_mp_[i] + move)
				{
					controller()->motorPool()[i].setTargetPos(current_angle[i] - move);
				}
			}
		};

        // 检查电机位置是否到达目标位置
		auto motorsPositionCheck = [](const double* current_sa_angle_, const double* target_pos_, size_t dim_)
		{
			for (int i = 0; i < dim_; i++)
			{
				if (std::fabs(current_sa_angle_[i] - target_pos_[i]) >= tolerance)
				{
					return false;
				}
			}

			return true;
		};

        // 单臂运动控制函数，根据目标位置求解逆运动学，并设置电机目标位置
		auto saMove = [&](double* pos_, aris::dynamic::Model& model_, int type_) 
        {
			model_.setOutputPos(pos_);
			if (model_.inverseKinematics())
			{
				throw std::runtime_error("Inverse Kinematics Position Failed!");
			}
			double x_joint[6]{ 0 };
			model_.getInputPos(x_joint);
			if (type_ == 0)
			{
				for (std::size_t i = 0; i < 6; ++i)
				{
					controller()->motorPool()[i].setTargetPos(x_joint[i]);
				}
			}
			else if (type_ == 1)
			{
				for (std::size_t i = 0; i < 6; ++i)
				{
					controller()->motorPool()[i + 6].setTargetPos(x_joint[i]);
				}
			}
			else
			{
				throw std::runtime_error("Arm Type Error");
			}
		};

        // 滤波函数，使用简单的平均滤波器对力传感器数据进行滤波
		auto forceFilter = [&](double* actual_force_, double* filtered_force_)
		{
			for (int i = 0; i < 6; i++)
			{
				imp_->force_buffer[i][imp_->buffer_index[i]] = actual_force_[i];
				imp_->buffer_index[i] = (imp_->buffer_index[i] + 1) % 10;
				filtered_force_[i] = std::accumulate(imp_->force_buffer[i].begin(), imp_->force_buffer[i].end(), 0.0) / 10;
			}
		};

        // 模拟外力作用
		for (std::size_t i = 0; i < 6; ++i)
		{
			imp_->actual_force[i] = 0;
		}
		if (count() > 2000 && count() <= 4000)
		{
			imp_->actual_force[3] = 0.5;
		}
		else if (count() > 5000 && count() <= 8000)
		{
			imp_->actual_force[0] = 5;
		}
        // 每个count周期读取一次电机位置
		for (int i = 0; i < 12; i++)
		{
			current_angle[i] = controller()->motorPool()[i].actualPos();
		}
        // 执行初始化，运动至初始位置，并将其设置为期望位置
		if (!imp_->init)
		{
			dualArm.setInputPos(init_angle);
			if (dualArm.forwardKinematics())
			{
				throw std::runtime_error("Forward Kinematics Position Failed!");
			}
			daJointMove(init_angle);
			if (motorsPositionCheck(current_angle, init_angle, 12))
			{
				eeA1.getP(imp_->arm1_x_d);
				eeA2.getP(imp_->arm2_x_d);
                //实机控制时在此处读取初始力偏置
				//getForceData(imp_->arm1_init_force, 0, imp_->init);
				//getForceData(imp_->arm2_init_force, 1, imp_->init);
				mout() << "Back To Init" << std::endl;
				imp_->init = true;
			}
		}
        // 确认接触
		else if (imp_->init && !imp_->contact_check)
		{
			double raw_force_checker[12]{ 0 };
			double comp_force_checker[12]{ 0 };
			double force_checker[12]{ 0 };
			double a1_pm[16]{ 0 };
			double a2_pm[16]{ 0 };
			eeA1.getMpm(a1_pm);
			eeA2.getMpm(a2_pm);
			//Arm1
			// getForceData(raw_force_checker, 0, imp_->init);
			// gc.getCompFT(a1_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, comp_force_checker);
			//Arm2
			// getForceData(raw_force_checker + 6, 1, imp_->init);
			// gc.getCompFT(a2_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, comp_force_checker + 6);

			for (int i = 0; i < 12; i++)
			{
				force_checker[i] = comp_force_checker[i] + raw_force_checker[i];
				if (abs(force_checker[i]) > 3) 
				{
					imp_->contact_check = true;
					mout() << "Contact Check" << std::endl;
					break;
				}
			}
            // 方便测试直接跳过接触检查
            imp_->contact_check = true;
		}
        // 如果接触检查通过，开始执行力保持控制
		else
		{
            // 单独控制臂1（白色）
			if (imp_->m_ == 0)
			{
                // 获取当前末端位姿（321欧拉）
				eeA1.getP(current_pos);
                // 获取当前末端位姿矩阵
				eeA1.getMpm(current_pm);
                // 获取原始力数据
				// getForceData(current_force, 0, imp_->init);
                // 重力补偿
				// gc.getCompFT(current_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, comp_force);
				// for (int i = 0; i < 6; i++)
				// {
				// 	actual_force[i] = comp_force[i] + current_force[i];
				// }
				// 力死区
				for (int i = 0; i < 6; i++)
				{
					if (abs(actual_force[i]) < trigger_force[i])
					{
						actual_force[i] = 0;
					}
					if (actual_force[i] > max_force[i])
					{
						actual_force[i] = max_force[i];
					}
					if (actual_force[i] < -max_force[i])
					{
						actual_force[i] = -max_force[i];
					}

				}
                // 力滤波
				forceFilter(actual_force, filtered_force);
				// 将力数据坐标变换到Arm1基坐标系下
				transform_force[0] = filtered_force[2];
				transform_force[1] = -filtered_force[1];
				transform_force[2] = filtered_force[0];
				transform_force[3] = filtered_force[5];
				transform_force[4] = -filtered_force[4];
				transform_force[5] = filtered_force[3];

                // 阻抗控制器
				double acc[3]{ 0 };
				double ome[3]{ 0 };
				double pm[16]{ 0 };
				double dx[3]{ 0 };
				double dth[3]{ 0 };
				double dt = 0.001;
				// 位置更新计算
				for (int i = 0; i < 3; i++)
				{
					// da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
					acc[i] = (-imp_->f_d[i] + imp_->actual_force[i] - imp_->a1_B[i] * (imp_->v_c[i] - imp_->v_d[i]) - imp_->a1_K[i] * (current_pos[i] - imp_->arm1_x_d[i])) / imp_->a1_M[i];
				}
				for (int i = 0; i < 3; i++)
				{
					imp_->v_c[i] += acc[i] * dt;
					dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
					current_pos[i] = dx[i] + current_pos[i];
				}
                // 姿态更新计算
				double rm_c[9]{ 0 };
				double rm_d[9]{ 0 };
				double rm_e[9]{ 0 };
				double pose_error[3]{ 0 };
				// 将期望位置的姿态（321欧拉表示）转换为旋转矩阵表示
				aris::dynamic::s_re2rm(imp_->arm1_x_d + 3, rm_d, "321");
				// 将当前末端位姿的姿态（321欧拉表示）转换为旋转矩阵表示
				aris::dynamic::s_re2rm(current_pos + 3, rm_c, "321");
				// 矩阵乘法计算旋转矩阵之间的位置偏差
				aris::dynamic::s_rm_dot_inv_rm(rm_c, rm_d, rm_e);
				// 将旋转矩阵转换为轴角表示
				aris::dynamic::s_rm2ra(rm_e, pose_error);
				// 使用轴角更新姿态
				for (int i = 0; i < 3; i++)
				{
					// Caculate Omega
					ome[i] = (-imp_->f_d[i + 3] + imp_->actual_force[i + 3] - imp_->a1_B[i + 3] * (imp_->v_c[i + 3] - imp_->v_d[i + 3]) - imp_->a1_K[i + 3] * pose_error[i]) / imp_->a1_M[i + 3];
				}
				for (int i = 0; i < 3; i++)
				{
					// Angluar Velocity
					imp_->v_c[i + 3] += ome[i] * dt;
					dth[i] = imp_->v_c[i + 3] * dt;
				}
				double drm[9]{ 0 };
				double rm_target[9]{ 0 };
				// 轴角转换为旋转矩阵
				aris::dynamic::s_ra2rm(dth, drm);
				// 计算下一步位置的旋转矩阵
				aris::dynamic::s_mm(3, 3, 3, drm, rm_c, rm_target);
                // 旋转矩阵转换为321欧拉角表示，并更新末端位姿
				aris::dynamic::s_rm2re(rm_target, current_pos + 3, "321");
                // 更新下一时刻末端位置
				saMove(current_pos, model_a1, 0);
                if (count() % 100 == 0)
				{
					mout() << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
						<< current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;
				}
			}
            // 单独控制臂2（蓝色）
			else if (imp_->m_ == 1)
			{
				eeA2.getP(current_pos);
				eeA2.getMpm(current_pm);
				//getForceData(current_force, 1, imp_->init);
				//gc.getCompFT(current_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, comp_force);
				//for (int i = 0; i < 6; i++)
				//{
				//	actual_force[i] = comp_force[i] + current_force[i];
				//}
				//Dead Zone of Force
				for (int i = 0; i < 6; i++)
				{
					if (abs(actual_force[i]) < trigger_force[i])
					{
						actual_force[i] = 0;
					}
					if (actual_force[i] > max_force[i])
					{
						actual_force[i] = max_force[i];
					}
					if (actual_force[i] < -max_force[i])
					{
						actual_force[i] = -max_force[i];
					}

				}
				forceFilter(actual_force, filtered_force);
				//Coordinate Transform Arm2
				transform_force[0] = -filtered_force[2];
				transform_force[1] = filtered_force[1];
				transform_force[2] = filtered_force[0];
				transform_force[3] = -filtered_force[5];
				transform_force[4] = filtered_force[4];
				transform_force[5] = filtered_force[3];

				double acc[3]{ 0 };
				double ome[3]{ 0 };
				double pm[16]{ 0 };
				double dx[3]{ 0 };
				double dth[3]{ 0 };
				double dt = 0.001;
				//position
				for (int i = 0; i < 3; i++)
				{
					// da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
					acc[i] = (-imp_->f_d[i] + imp_->actual_force[i] - imp_->a2_B[i] * (imp_->v_c[i] - imp_->v_d[i]) - imp_->a2_K[i] * (current_pos[i] - imp_->arm2_x_d[i])) / imp_->a2_M[i];
				}
				for (int i = 0; i < 3; i++)
				{
					imp_->v_c[i] += acc[i] * dt;
					dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
					current_pos[i] = dx[i] + current_pos[i];

				}
				double rm_c[9]{ 0 };
				double rm_d[9]{ 0 };
				double rm_e[9]{ 0 };
				double pose_error[3]{ 0 };
				//Rm of Desired Pos
				aris::dynamic::s_re2rm(imp_->arm2_x_d + 3, rm_d, "321");
				//Current pe to rm
				aris::dynamic::s_re2rm(current_pos + 3, rm_c, "321");
				//Inverse
				aris::dynamic::s_rm_dot_inv_rm(rm_c, rm_d, rm_e);
				//Convert Rm to Ra
				aris::dynamic::s_rm2ra(rm_e, pose_error);
				//pose
				for (int i = 0; i < 3; i++)
				{
					// Caculate Omega
					ome[i] = (-imp_->f_d[i + 3] + imp_->actual_force[i + 3] - imp_->a2_B[i + 3] * (imp_->v_c[i + 3] - imp_->v_d[i + 3]) - imp_->a2_K[i + 3] * pose_error[i]) / imp_->a2_M[i + 3];
				}
				for (int i = 0; i < 3; i++)
				{
					// Angluar Velocity
					imp_->v_c[i + 3] += ome[i] * dt;
					dth[i] = imp_->v_c[i + 3] * dt;
				}
				double drm[9]{ 0 };
				double rm_target[9]{ 0 };
				//Transform to rm
				aris::dynamic::s_ra2rm(dth, drm);
				//Calcuate Future rm
				aris::dynamic::s_mm(3, 3, 3, drm, rm_c, rm_target);
				//Convert rm to pe
				aris::dynamic::s_rm2re(rm_target, current_pos + 3, "321");
				saMove(current_pos, model_a2, 1);
                if (count() % 100 == 0)
				{
					mout() << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
						<< current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;
				}
			}
			else
			{
				mout() << "Wrong Model" << std::endl;
				return 0;
			}
		}
		return 30000 - count();
	}
	ForceKeep::ForceKeep(const std::string& name)
	{
		aris::core::fromXmlString(command(),
			"<Command name=\"m_fk\">"
			"	<GroupParam>"
			"	<Param name=\"model\" default=\"0\" abbreviation=\"m\"/>"
			"	</GroupParam>"
			"</Command>");
	}
	ForceKeep::~ForceKeep() = default;
    
    // ForceDrag指令模拟导纳控制
	struct ForceDrag::Imp {
			//Flag
			bool init = false;
			bool contact_check = false;
			//Force Compensation Parameter
			double comp_f[6]{ 0 };
			//Arm1
			double arm1_init_force[6]{ 0 };
			double arm1_p_vector[6]{ 0 };
			double arm1_l_vector[6]{ 0 };
			//Arm2
			double arm2_init_force[6]{ 0 };
			double arm2_p_vector[6]{ 0 };
			double arm2_l_vector[6]{ 0 };
			//Desired Pos, Vel, Acc, Foc
			double arm1_x_d[6]{ 0 };
			double arm2_x_d[6]{ 0 };
			double v_d[6]{ 0 };
			double a_d[6]{ 0 };
			double f_d[6]{ 0 };
			//Current Vel
			double v_c[6]{ 0 };
			//Impedence Parameter
			//double K[6]{ 100,100,100,100,100,100 };
			double B[6]{ 100,100,100,5,5,2 };
			double M[6]{ 100,100,100,5,5,2 };
			double Ke[6]{ 220000,220000,220000,220000,220000,220000 };
			//Counter
			int contact_count = 0;
			//Test
			double actual_force[6]{ 0 };
			//Switch Model
			int m_;
			//Force Buffer
			std::array<double, 10> force_buffer[6] = {};
			int buffer_index[6]{ 0 };
		};
	auto ForceDrag::prepareNrt() -> void
	{
		for (auto& m : motorOptions()) m =
			aris::plan::Plan::CHECK_NONE |
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
		// GravComp gc;
		// gc.loadPLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);
		mout() << "Load P & L Vector" << std::endl;
	}
	auto ForceDrag::executeRT() -> int
	{
		//dual transform modelbase into multimodel
		auto& dualArm = dynamic_cast<aris::dynamic::MultiModel&>(modelBase()[0]);
		//at(0) -> Arm1 -> white
		auto& arm1 = dualArm.subModels().at(0);
		//at(1) -> Arm2 -> blue
		auto& arm2 = dualArm.subModels().at(1);
		//transform to model
		auto& model_a1 = dynamic_cast<aris::dynamic::Model&>(arm1);
		auto& model_a2 = dynamic_cast<aris::dynamic::Model&>(arm2);
		//End Effector
		auto& eeA1 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a1.generalMotionPool().at(0));
		auto& eeA2 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a2.generalMotionPool().at(0));
		//ver 1.0 not limit on vel, only limit force
		static double tolerance = 0.0001;
		static double init_angle[12] =
		{ 0, 0, 5 * PI / 6, -5 * PI / 6, -PI / 2, 0 ,
		0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, 0 };
		static double max_vel[6]{ 0.2,0.2,0.2,0.0005,0.0005,0.0001 };
		static double trigger_force[6]{ 0.5,0.5,0.5,0.001,0.001,0.001 };
		static double max_force[6]{ 10,10,10,5,5,5 };
		static double trigger_vel[6]{ 0.0001,0.0001,0.0001,0.0001,0.0001,0.0001 };

		GravComp gc;
		double current_angle[12]{ 0 };
		double current_sa_angle[6]{ 0 };
		double comp_force[6]{ 0 };
		double current_pm[16]{ 0 };
		double current_pos[6]{ 0 };
		double current_force[6]{ 0 };
		double actual_force[6]{ 0 };
		double filtered_force[6]{ 0 };
		double transform_force[6]{ 0 };

		imp_->m_ = int32Param("model");
		auto getForceData = [&](double* data_, int m_, bool init_)
		{
			int raw_force[6]{ 0 };
			for (std::size_t i = 0; i < 6; ++i)
			{
				if (ecMaster()->slavePool()[9 + 9 * m_].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
					mout() << "error" << std::endl;
				data_[i] = (static_cast<double>(raw_force[i]) / 1000.0);
			}
			if (!init_)
			{
				mout() << "Compensate Init Force" << std::endl;
			}
			else
			{
				if (m_ == 0)
				{
					for (std::size_t i = 0; i < 6; ++i)
					{
						data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm1_init_force[i];
					}
				}
				else if (m_ == 1)
				{
					for (std::size_t i = 0; i < 6; ++i)
					{
						data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm2_init_force[i];
					}
				}
				else
				{
					mout() << "Wrong Model" << std::endl;
				}
			}
		};

		auto daJointMove = [&](double target_mp_[12])
		{
			double current_angle[12] = { 0 };
			double move = 0.00005;
			for (int i = 0; i < 12; i++)
			{
				current_angle[i] = controller()->motorPool()[i].targetPos();
			}
			for (int i = 0; i < 12; i++)
			{
				if (current_angle[i] <= target_mp_[i] - move)
				{
					controller()->motorPool()[i].setTargetPos(current_angle[i] + move);
				}
				else if (current_angle[i] >= target_mp_[i] + move)
				{
					controller()->motorPool()[i].setTargetPos(current_angle[i] - move);
				}
			}
		};

		auto motorsPositionCheck = [](const double* current_sa_angle_, const double* target_pos_, size_t dim_)
		{
			for (int i = 0; i < dim_; i++)
			{
				if (std::fabs(current_sa_angle_[i] - target_pos_[i]) >= tolerance)
				{
					return false;
				}
			}
			return true;
		};
		
		auto saMove = [&](double* pos_, aris::dynamic::Model& model_, int type_) {
			model_.setOutputPos(pos_);
			if (model_.inverseKinematics())
			{
				throw std::runtime_error("Inverse Kinematics Position Failed!");
			}
			double x_joint[6]{ 0 };
			model_.getInputPos(x_joint);
			if (type_ == 0)
			{
				for (std::size_t i = 0; i < 6; ++i)
				{
					controller()->motorPool()[i].setTargetPos(x_joint[i]);
				}
			}
			else if (type_ == 1)
			{
				for (std::size_t i = 0; i < 6; ++i)
				{
					controller()->motorPool()[i + 6].setTargetPos(x_joint[i]);
				}
			}
			else
			{
				throw std::runtime_error("Arm Type Error");
			}
		};

		auto forceFilter = [&](double* actual_force_, double* filtered_force_)
		{
			for (int i = 0; i < 6; i++)
			{
				imp_->force_buffer[i][imp_->buffer_index[i]] = actual_force_[i];
				imp_->buffer_index[i] = (imp_->buffer_index[i] + 1) % 10;

				filtered_force_[i] = std::accumulate(imp_->force_buffer[i].begin(), imp_->force_buffer[i].end(), 0.0) / 10;
			}
		};
        
        //模拟外力作用
		for (std::size_t i = 0; i < 6; ++i)
		{
			imp_->actual_force[i] = 0;
		}
		if (count() > 2000 && count() <= 5000)
		{
			//mout() << "fex" << std::endl;
			imp_->actual_force[0] = -5;
		}
		else if (count() > 5100 && count() <= 8000)
		{
			//mout() << "fex" << std::endl;
			imp_->actual_force[2] = 5;
		}

		for (int i = 0; i < 12; i++)
		{
			current_angle[i] = controller()->motorPool()[i].actualPos();
		}
		if (!imp_->init)
		{
			dualArm.setInputPos(init_angle);
			if (dualArm.forwardKinematics())
			{
				throw std::runtime_error("Forward Kinematics Position Failed!");
			}
			// Test
			for (std::size_t i = 0; i < 12; ++i)
			{
				controller()->motorPool()[i].setTargetPos(init_angle[i]);
			}

			daJointMove(init_angle);
			if (motorsPositionCheck(current_angle, init_angle, 12))
			{
				eeA1.getP(imp_->arm1_x_d);
				eeA2.getP(imp_->arm2_x_d);
				mout() << imp_->arm1_x_d[0] << '\t' << imp_->arm1_x_d[1] << '\t' << imp_->arm1_x_d[2] << '\t'
					<< imp_->arm1_x_d[3] << '\t' << imp_->arm1_x_d[4] << '\t' << imp_->arm1_x_d[5] << std::endl;
				//getForceData(imp_->arm1_init_force, 0, imp_->init);
				//getForceData(imp_->arm2_init_force, 1, imp_->init);
				mout() << "Back To Init" << std::endl;
				imp_->init = true;
			}
		}
		else if (imp_->init && !imp_->contact_check)
		{
			double raw_force_checker[12]{ 0 };
			double comp_force_checker[12]{ 0 };
			double force_checker[12]{ 0 };
			double a1_pm[16]{ 0 };
			double a2_pm[16]{ 0 };
			eeA1.getMpm(a1_pm);
			eeA2.getMpm(a2_pm);
			// //Arm1
			// getForceData(raw_force_checker, 0, imp_->init);
			// gc.getCompFT(a1_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, comp_force_checker);
			// //Arm2
			// getForceData(raw_force_checker + 6, 1, imp_->init);
			// gc.getCompFT(a2_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, comp_force_checker + 6);

			for (int i = 0; i < 12; i++)
			{
				force_checker[i] = comp_force_checker[i] + raw_force_checker[i];
				if (abs(force_checker[i]) > 3)
				{
					imp_->contact_check = true;
					mout() << "Contact Check" << std::endl;
					break;
				}
			}
            // 方便测试直接跳过接触检查
            imp_->contact_check = true;
		}
		else
		{
			if (imp_->m_ == 0)
			{
				double current_vel[6]{ 0 };
				eeA1.getP(current_pos);
				eeA1.getV(current_vel);
				eeA1.getMpm(current_pm);
				//getForceData(current_force, 0, imp_->init);
				//gc.getCompFT(current_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, comp_force);
				//for (int i = 0; i < 6; i++)
				//{
				//	actual_force[i] = comp_force[i] + current_force[i];
				//}
                forceFilter(actual_force, filtered_force);
                //Coordinate Transform Arm1
				transform_force[0] = filtered_force[2];
				transform_force[1] = -filtered_force[1];
				transform_force[2] = filtered_force[0];
				transform_force[3] = filtered_force[5];
				transform_force[4] = -filtered_force[4];
				transform_force[5] = filtered_force[3];

                // 导纳控制器，没有位置项
				double acc[3]{ 0 };
				double ome[3]{ 0 };
				double pm[16]{ 0 };
				double dx[3]{ 0 };
				double dth[3]{ 0 };
				double dt = 0.001;
				//position
				for (int i = 0; i < 3; i++)
				{
					// da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
					acc[i] = (-imp_->f_d[i] + imp_->actual_force[i] - imp_->B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->M[i];
				}
				for (int i = 0; i < 3; i++)
				{
					imp_->v_c[i] += acc[i] * dt;
					dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
					current_pos[i] = dx[i] + current_pos[i];

				}
				//pose
				for (int i = 0; i < 3; i++)
				{
					// Caculate Omega
					ome[i] = (-imp_->f_d[i + 3] + imp_->actual_force[i + 3] - imp_->B[i + 3] * (imp_->v_c[i + 3] - imp_->v_d[i + 3])) / imp_->M[i + 3];
				}
				for (int i = 0; i < 3; i++)
				{
					// Angluar Velocity
					imp_->v_c[i + 3] += ome[i] * dt;
					dth[i] = imp_->v_c[i + 3] * dt;
				}
				double drm[9]{ 0 };
				double rm_target[9]{ 0 };
				double rm_c[9]{ 0 };
				//Transform to rm
				aris::dynamic::s_ra2rm(dth, drm);
				//Current pe to rm
				aris::dynamic::s_re2rm(current_pos + 3, rm_c, "321");
				//Calcuate Future rm
				aris::dynamic::s_mm(3, 3, 3, drm, rm_c, rm_target);
				//Convert rm to pe
				aris::dynamic::s_rm2re(rm_target, current_pos + 3, "321");
				saMove(current_pos, model_a1, 0);
				if (count() % 100 == 0)
				{
					mout() << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
						<< current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;
				}
			}
			else if (imp_->m_ == 1)
			{
				double current_vel[6]{ 0 };
				eeA2.getP(current_pos);
				eeA2.getV(current_vel);
				eeA2.getMpm(current_pm);
				//getForceData(current_force, 1, imp_->init);
				//gc.getCompFT(current_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, comp_force);
				//for (int i = 0; i < 6; i++)
				//{
				//	actual_force[i] = comp_force[i] + current_force[i];
				//}
                forceFilter(actual_force, filtered_force);
				//Coordinate Transform Arm2
				transform_force[0] = -filtered_force[2];
				transform_force[1] = filtered_force[1];
				transform_force[2] = filtered_force[0];
				transform_force[3] = -filtered_force[5];
				transform_force[4] = filtered_force[4];
				transform_force[5] = filtered_force[3];

				double acc[3]{ 0 };
				double ome[3]{ 0 };
				double pm[16]{ 0 };
				double dx[3]{ 0 };
				double dth[3]{ 0 };
				double dt = 0.001;
				//position
				for (int i = 0; i < 3; i++)
				{
					// da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
					acc[i] = (-imp_->f_d[i] + imp_->actual_force[i] - imp_->B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->M[i];
				}
				for (int i = 0; i < 3; i++)
				{
					imp_->v_c[i] += acc[i] * dt;
					dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
					current_pos[i] = dx[i] + current_pos[i];

				}
				//pose
				for (int i = 0; i < 3; i++)
				{
					// Caculate Omega
					ome[i] = (-imp_->f_d[i + 3] + imp_->actual_force[i + 3] - imp_->B[i + 3] * (imp_->v_c[i + 3] - imp_->v_d[i + 3])) / imp_->M[i + 3];
				}
				for (int i = 0; i < 3; i++)
				{
					// Angluar Velocity
					imp_->v_c[i + 3] += ome[i] * dt;
					dth[i] = imp_->v_c[i + 3] * dt;
				}
				double drm[9]{ 0 };
				double rm_target[9]{ 0 };
				double rm_c[9]{ 0 };
				//Transform to rm
				aris::dynamic::s_ra2rm(dth, drm);
				//Current pe to rm
				aris::dynamic::s_re2rm(current_pos + 3, rm_c, "321");
				//Calcuate Future rm
				aris::dynamic::s_mm(3, 3, 3, drm, rm_c, rm_target);
				//Convert rm to pe
				aris::dynamic::s_rm2re(rm_target, current_pos + 3, "321");
				saMove(current_pos, model_a2, 1);
				if (count() % 100 == 0)
				{
					mout() << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
						<< current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;
				}
			}
			else
			{
				mout() << "Wrong Model" << std::endl;
				return 0;
			}
		}
		return 10000 - count();
	}
	ForceDrag::ForceDrag(const std::string& name)
	{
		aris::core::fromXmlString(command(),
			"<Command name=\"m_fd\">"
			"	<GroupParam>"
			"	<Param name=\"model\" default=\"0\" abbreviation=\"m\"/>"
			"	</GroupParam>"
			"</Command>");
	}
	ForceDrag::~ForceDrag() = default;

	ARIS_REGISTRATION{
		aris::core::class_<ForceKeep>("ForceKeep")
			.inherit<aris::plan::Plan>();
		aris::core::class_<ForceDrag>("ForceDrag")
			.inherit<aris::plan::Plan>();
	}
}





