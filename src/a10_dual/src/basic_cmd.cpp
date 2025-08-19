#include "basic_cmd.hpp"
#include "t_curve.hpp"
#include <cmath>

using namespace std;
const double PI = 3.141592653589793;

namespace basic_cmd
{	
	// 正逆运动学设置位置示例指令
	// PrepareNrt函数在非实时线程中执行，通常用于执行初始设置，例如设置电机选项等
	auto ModelSetPos::prepareNrt()->void
	{
		// 设置电机选项，禁用位置二阶连续性检查，取消所有检查（check_none）
        // 注意！！取消所有检查会才能在仿真中运行，实机控制不能取消所有检查！！！会导致忽略必要的使能，电流过载检查
        for (auto& m : motorOptions()) m = aris::plan::Plan::CHECK_NONE |
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
	}
	auto ModelSetPos::executeRT()->int
	{
		// aris中所有初始化的模型（xml中定义）都存储在modelBase()中，ModelBase是所有模型的基类，有一些针对具体模型的操作无法直接对ModelBase使用，因此需要进行类型转换
		// 首先将modelBase转换为MultiModel类型，MultiModel是一个包含多个子模型的模型类
		// 可以直接对MultiModel进行正逆运动学等操作
		auto& dualArm = dynamic_cast<aris::dynamic::MultiModel&>(modelBase()[0]);
		// 由于本例子中定义的是包含两个机械臂的双臂机器人，因此可以通过subModels()方法获取到这两个子模型
		// at(0) -> Arm1 -> white
		auto& arm1 = dualArm.subModels().at(0);
		// at(1) -> Arm2 -> blue
		auto& arm2 = dualArm.subModels().at(1);
		// 再将其转换为具体的Model类型，以便进行更具体的操作
		auto& model_a1 = dynamic_cast<aris::dynamic::Model&>(arm1);
		auto& model_a2 = dynamic_cast<aris::dynamic::Model&>(arm2);
		// 获取末端执行器指针
		auto& eeA1 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a1.generalMotionPool().at(0));
		auto& eeA2 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a2.generalMotionPool().at(0));
		// 例1：初始化位置
		static double init_angle[12] =
			{ 0, 0, 5 * PI / 6, -5 * PI / 6, -PI / 2, 0,
			0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, 0 };
		// 每一个指令内都是实时循环，循环次数由count()函数返回，count()从1开始计数，每次执行executeRT()函数时加1，目前仿真中一个count对应2ms（可以在xml中修改）
		if(count()==1)
		{
			// 可以直接对modelbase()进行一些操作，但不建议这么做
			modelBase()->setInputPos(init_angle);
			// 设置完后一定要求解正运动学
			if (modelBase()->forwardKinematics())std::cout << "forward failed" << std::endl;
			// 模型的位置跟电机的位置是分开的，设置完模型的位置后，需要将电机的位置设置为目标位置
			for(int i = 0; i<12; i++)
			{
				controller()->motorPool()[i].setTargetPos(init_angle[i]);
			}
			// 设置双臂的输入位置为初始角度
			dualArm.setInputPos(init_angle);
			// 设置完后一定要求解正运动学，才能将输入位置转换为末端执行器的位置
			if (dualArm.forwardKinematics())std::cout << "forward failed" << std::endl;
			for(int i = 0; i<12; i++)
			{
				controller()->motorPool()[i].setTargetPos(init_angle[i]);
			}
			// 求解完正运动学后，可以获取当前双臂的末端执行器位置
			double dualarm_current_pos[12]{0};
			dualArm.getOutputPos(dualarm_current_pos);
			// aris中输出尽量使用mout，避免影响实时性，但有时候系统段错误时mout可能无法输出，此时还是用std::cout
			mout()<<"current dualarm pos:"<< '\n' << dualarm_current_pos[0] << '\t' << dualarm_current_pos[1] << '\t' 
				<< dualarm_current_pos[2] << '\t' << dualarm_current_pos[3] << '\t' 
				<< dualarm_current_pos[4] << '\t' << dualarm_current_pos[5] << '\n'
				<< dualarm_current_pos[6] << std::endl;

			// 逆解流程一致
			dualArm.setOutputPos(dualarm_current_pos);
			if (dualArm.inverseKinematics())std::cout << "inverse failed" << std::endl;
			double dualarm_input_angle[12]{0};
			dualArm.getInputPos(dualarm_input_angle);
			mout()<<"current dualarm input angle:"<< '\n' << dualarm_input_angle[0] << '\t' << dualarm_input_angle[1] << '\t' 
				<< dualarm_input_angle[2] << '\t' << dualarm_input_angle[3] << '\t' 
				<< dualarm_input_angle[4] << '\t' << dualarm_input_angle[5] << '\n'
				<< dualarm_input_angle[6] << std::endl;

			// 相应的，可以对单个机械臂模型进行正逆运动学操作
			double arm1_input_angle[6]{0};
			std::copy(dualarm_input_angle, dualarm_input_angle + 6, arm1_input_angle);

			model_a1.setInputPos(arm1_input_angle);
			if (model_a1.forwardKinematics())std::cout << "arm1 forward failed" << std::endl;

			double arm1_output_pos[6]{0};
			model_a1.getOutputPos(arm1_output_pos);
			mout()<<"current arm1 output pos:"<< '\n' << arm1_output_pos[0] << '\t' << arm1_output_pos[1] << '\t' 
				<< arm1_output_pos[2] << '\t' << arm1_output_pos[3] << '\t' 
				<< arm1_output_pos[4] << '\t' << arm1_output_pos[5] << std::endl;

			model_a1.setOutputPos(arm1_output_pos);
			if (model_a1.inverseKinematics())std::cout << "arm1 inverse failed" << std::endl;

			double arm1_input_pos[6]{0};
			model_a1.getInputPos(arm1_input_pos);
			mout()<<"current arm1 input angle:"<< '\n' << arm1_input_pos[0] << '\t' << arm1_input_pos[1] << '\t' 
				<< arm1_input_pos[2] << '\t' << arm1_input_pos[3] << '\t' 
				<< arm1_input_pos[4] << '\t' << arm1_input_pos[5] << std::endl;

			// 也可以直接通过ee进行控制，值得注意的是，ee对应的末端坐标系是模型的基坐标系，并非传统意义上的末端执行器坐标系
			eeA1.setP(arm1_output_pos);
			if (model_a1.inverseKinematics())std::cout << "arm1 ee inverse failed" << std::endl;
			double ee1_input_pos[6]{0};
			eeA1.getP(ee1_input_pos);
			mout()<<"current ee1 input pos:"<< '\n' << ee1_input_pos[0] << '\t' << ee1_input_pos[1] << '\t' 
				<< ee1_input_pos[2] << '\t' << ee1_input_pos[3] << '\t' 
				<< ee1_input_pos[4] << '\t' << ee1_input_pos[5] << std::endl;
			// ee能支持的操作多一些，比如直接获取末端执行器的位姿矩阵
			double ee1_output_pm[16]{0};
			eeA1.getMpm(ee1_output_pm);
			mout()<<"current ee1 output pm:"<< '\n' << ee1_output_pm[0] << '\t' << ee1_output_pm[1] << '\t' 
				<< ee1_output_pm[2] << '\t' << ee1_output_pm[3] << '\n'
				<< ee1_output_pm[4] << '\t' << ee1_output_pm[5] << '\t' 
				<< ee1_output_pm[6] << '\t' << ee1_output_pm[7] << '\n'
				<< ee1_output_pm[8] << '\t' << ee1_output_pm[9] << '\t' 
				<< ee1_output_pm[10] << '\t' << ee1_output_pm[11] << '\n'
				<< ee1_output_pm[12] << '\t' << ee1_output_pm[13] << '\t' 
				<< ee1_output_pm[14] << '\t' << ee1_output_pm[15] << std::endl;

		}
		mout()<<"count"<<'\n'<<count()<<std::endl;
		// 此处决定循环终止时间
		return 10 - count();
	}
	ModelSetPos::ModelSetPos(const std::string& name)
	{	//此处注册命令的xml字符串，定义的指令名称即为终端输入时的名称
		aris::core::fromXmlString(command(),
			"<Command name=\"m_set\"/>");
	}
	ModelSetPos::~ModelSetPos() = default;

	// 单臂逆运动学运动示例指令
	// 结构体内数据不会随着循环而重新初始化
	struct ModelForward::Imp {
			bool init = false;
		};
	auto ModelForward::prepareNrt()->void
	{
		for (auto& m : motorOptions()) m =
			aris::plan::Plan::CHECK_NONE |
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
	}
	auto ModelForward::executeRT()->int
	{
		//dual transform modelbase into multimodel
		auto& dualArm = dynamic_cast<aris::dynamic::MultiModel&>(modelBase()[0]);
		//at(0) -> Arm1 ->white
		auto& arm1 = dualArm.subModels().at(0);
		//at(1) -> Arm2 ->blue
		auto& arm2 = dualArm.subModels().at(1);
		//transform to model
		auto& model_a1 = dynamic_cast<aris::dynamic::Model&>(arm1);
		auto& model_a2 = dynamic_cast<aris::dynamic::Model&>(arm2);
		auto& eeA1 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a1.generalMotionPool().at(0));
		auto& eeA2 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a2.generalMotionPool().at(0));
		static double tolerance = 0.0001;
		static double init_angle[12] = 
		{ 0, 0, 5 * PI / 6, -5 * PI / 6, - PI / 2, 0, 
		0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, 0 };

		double current_angle[12]{0};
		// 双臂电机移动lambda函数
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
		// 电机位置检查lambda函数
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

		// 循环初始获取当前电机位置
        for (int i = 0; i < 12; i++)
		{
			current_angle[i] = controller()->motorPool()[i].actualPos();
		}
		// 判断是否回到初始位置
		if(!imp_->init)
		{
			dualArm.setInputPos(init_angle);
			if(dualArm.forwardKinematics())
			{
				std::cout<<"Forward Error"<<std::endl;
			}
			daJointMove(init_angle);
			if(motorsPositionCheck(current_angle,init_angle,12))
			{
				mout()<<"Init Complete"<<std::endl;
				imp_->init = true;
			}
		}
		//仅控制臂1向前移动
		else
		{	//Only Arm 1
			double current_pos[6]{0};
			eeA1.getP(current_pos);
			// aris中单位是米，每个count运动步长过大实机控制时会报错
			current_pos[0] +=0.00001;
			if (count() % 1000 == 0)
			{
				mout() << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
					<< current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;
			}
			saMove(current_pos, model_a1, 0);
		}
		return 8000 - count();
	}
	ModelForward::ModelForward(const std::string& name)
	{
        aris::core::fromXmlString(command(),
            "<Command name=\"m_forward\">"
            "</Command>");
	}
	ModelForward::~ModelForward() = default;

	// 将模型直接设置为初始位置指令
	auto ModelInit::prepareNrt()->void {

		for (auto& m : motorOptions()) m =
			aris::plan::Plan::CHECK_NONE |
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
	}
	auto ModelInit::executeRT()->int {
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

		static double init_pos[12] = 
		{ 0, 0, 5 * PI / 6, -5 * PI / 6, - PI / 2, 0, 
		0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, 0 };
		double current_pos_a1[6]{0};
		double current_pos_a2[6]{0};

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

		dualArm.setInputPos(init_pos);
		if (dualArm.forwardKinematics())
		{
			throw std::runtime_error("Forward Kinematics Position Failed!");
		}
		eeA1.getP(current_pos_a1);
		eeA2.getP(current_pos_a2);
		double current_angle[12] = { 0 };
        //Test
		for (int i = 0; i < 12; i++)
		{
			current_angle[i] = controller()->motorPool()[i].targetPos();
			controller()->motorPool()[i].setTargetPos(init_pos[i]);
		}
        // 方便调试直接设置位置，不模拟电机运动
		//daJointMove(init_pos);
		if (count() % 1000 == 0)
		{
			mout() << current_angle[0] << '\t' << current_angle[1] << '\t' << current_angle[2] << '\t' << current_angle[3] << '\t' << current_angle[4] << '\t'
				<< current_angle[5] << '\t' << current_angle[6] << '\t' << current_angle[7] << '\t' << current_angle[8] << '\t' << current_angle[9] << '\t'
				<< current_angle[10] << '\t' << current_angle[11] << '\t' << std::endl;
		}
		if (motorsPositionCheck(current_angle, init_pos, 12))
		{
			mout() << "Back to Init Position" << std::endl;
			dualArm.setInputPos(current_angle);
			if (dualArm.forwardKinematics())std::cout << "forward failed" << std::endl;

			mout() << "current angle: \n" << current_angle[0] <<'\t' << current_angle[1] << '\t' << current_angle[2] << '\t' << current_angle[3] << '\t' << current_angle[4] << '\t'
				<< current_angle[5] << '\t' << current_angle[6] << '\t' << current_angle[7] << '\t' << current_angle[8] << '\t' << current_angle[9] << '\t'
				<< current_angle[10] << '\t' << current_angle[11] << '\t' << std::endl;

			mout() << "current a1 pos: \n" << current_pos_a1[0] <<'\t' << current_pos_a1[1] << '\t' << current_pos_a1[2] << '\t' << current_pos_a1[3] << '\t' << current_pos_a1[4] << '\t'
				<< current_pos_a1[5] << std::endl;

			mout() << "current a2 pos: \n" << current_pos_a2[0] <<'\t' << current_pos_a2[1] << '\t' << current_pos_a2[2] << '\t' << current_pos_a2[3] << '\t' << current_pos_a2[4] << '\t'
				<< current_pos_a2[5] << std::endl;

			return 0;
		}
		else
		{
			if (count() == 80000)
			{
				mout() << "Over Time" << std::endl;
			}
			return 80000 - count();
		}
	}
	ModelInit::ModelInit(const std::string& name)
	{
		aris::core::fromXmlString(command(),
			"<Command name=\"m_init\"/>");
	}
	ModelInit::~ModelInit() = default;

	// T形曲线示例指令
	struct TCurveTest::Imp {
			bool init = false;
			//Switch Model
			int m_;
			int start_count = 0;
			double init_pos[6]{0};
		};
	auto TCurveTest::prepareNrt()->void
	{
		for (auto& m : motorOptions()) m =
			aris::plan::Plan::CHECK_NONE |
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
	}
	auto TCurveTest::executeRT()->int
	{
		//dual transform modelbase into multimodel
		auto& dualArm = dynamic_cast<aris::dynamic::MultiModel&>(modelBase()[0]);
		//at(0) -> Arm1 ->white
		auto& arm1 = dualArm.subModels().at(0);
		//at(1) -> Arm2 ->blue
		auto& arm2 = dualArm.subModels().at(1);
		//transform to model
		auto& model_a1 = dynamic_cast<aris::dynamic::Model&>(arm1);
		auto& model_a2 = dynamic_cast<aris::dynamic::Model&>(arm2);
		auto& eeA1 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a1.generalMotionPool().at(0));
		auto& eeA2 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a2.generalMotionPool().at(0));
		static double tolerance = 0.0001;
		static double init_angle[12] = 
		{ 0, 0, 5 * PI / 6, -5 * PI / 6, - PI / 2, 0, 
		0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, 0 };

		double current_angle[12]{0};
		// 双臂电机移动lambda函数
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
		// 电机位置检查lambda函数
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

		// 获取当前模型号
		imp_->m_ = int32Param("model");
		// 循环初始获取当前电机位置
        for (int i = 0; i < 12; i++)
		{
			current_angle[i] = controller()->motorPool()[i].actualPos();
		}
		// 判断是否回到初始位置
		if(!imp_->init)
		{
			dualArm.setInputPos(init_angle);
			if(dualArm.forwardKinematics())
			{
				std::cout<<"Forward Error"<<std::endl;
			}
			daJointMove(init_angle);
			if(motorsPositionCheck(current_angle,init_angle,12))
			{
				mout()<<"Init Complete"<<std::endl;
				imp_->init = true;
				// 记录开始时间
				imp_->start_count = count();
				if(imp_->m_ == 0)
				{
					// 仅控制臂1
					// 获取臂1末端执行器位置
					eeA1.getP(imp_->init_pos);
				}
				else if(imp_->m_ == 1)
				{
					// 仅控制臂2
					// 获取臂2末端执行器位置
					eeA2.getP(imp_->init_pos);
				}
				else
				{
					throw std::runtime_error("Model Type Error");
				}
			}
		}
		//仅控制臂1向前移动
		else
		{	
			int current_count = 0;
			current_count = count() - imp_->start_count;
			// 计算T形曲线位置
			TCurve c1(0.01,0.008,0.02);
			c1.getCurveParam();
			double pos = c1.getTCurve(current_count);
			// aris中单位是米，每个count运动步长过大实机控制时会报错
			double target_pos[6]{0};
			std::copy(imp_->init_pos, imp_->init_pos + 6, target_pos);
			target_pos[0] += pos;
			if (count() % 1000 == 0)
			{
				mout() << target_pos[0] << '\t' << target_pos[1] << '\t' << target_pos[2] << '\t'
					<< target_pos[3] << '\t' << target_pos[4] << '\t' << target_pos[5] << std::endl;
			}
			if(imp_->m_ == 0)
			{
				saMove(target_pos, model_a1, imp_->m_);
			}
			else if(imp_->m_ == 1)
			{
				saMove(target_pos, model_a2, imp_->m_);
			}
			else
			{
				throw std::runtime_error("Model Type Error");
			}
		}
		return 10000 - count();
	}
	TCurveTest::TCurveTest(const std::string& name)
	{
		aris::core::fromXmlString(command(),
			"<Command name=\"t_test\">"
			"	<GroupParam>"
			"	<Param name=\"model\" default=\"0\" abbreviation=\"m\"/>"
			"	</GroupParam>"
			"</Command>");
	}
	TCurveTest::~TCurveTest() = default;


	ARIS_REGISTRATION{
		aris::core::class_<ModelInit>("ModelInit")
			.inherit<aris::plan::Plan>();
		aris::core::class_<ModelForward>("ModelForward")
			.inherit<aris::plan::Plan>();
		aris::core::class_<ModelSetPos>("ModelSetPos")
			.inherit<aris::plan::Plan>();
		aris::core::class_<TCurveTest>("TCurveTest")
			.inherit<aris::plan::Plan>();
	}
}





