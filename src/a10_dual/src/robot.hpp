#ifndef PLAN_H
#define PLAN_H

#include <aris.hpp>

namespace robot
{



	class ModelSetPos :public aris::core::CloneObject<ModelSetPos, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~ModelSetPos();
		explicit ModelSetPos(const std::string& name = "ModelSetPos");
	};



	class ModelForward :public aris::core::CloneObject<ModelForward, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~ModelForward();
		explicit ModelForward(const std::string& name = "ModelForward");
	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};



	class ModelGet : public aris::core::CloneObject<ModelGet, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~ModelGet();
		explicit ModelGet(const std::string& name = "ModelGet");
		
	};


	class ModelInit : public aris::core::CloneObject<ModelInit, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~ModelInit();
		explicit ModelInit(const std::string& name = "ModelInit");
    private:
	};




	class ModelTest :public aris::core::CloneObject<ModelTest, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~ModelTest();
		explicit ModelTest(const std::string& name = "ModelTest");
	private:
		double cef_;
	};


	class ModelMoveX :public aris::core::CloneObject<ModelMoveX, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~ModelMoveX();
		explicit ModelMoveX(const std::string& name = "ModelMoveX");

	private:
		int m_;
		double d_;
		double o_;
	};


class ModelComP :public aris::core::CloneObject<ModelComP, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~ModelComP();
		explicit ModelComP(const std::string& name = "ModelComP");

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;

	};

class ForceAlign :public aris::core::CloneObject<ForceAlign, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~ForceAlign();
		explicit ForceAlign(const std::string& name = "ForceAlign");

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;

		

};




	class ForceKeep :public aris::core::CloneObject<ForceKeep, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~ForceKeep();
		explicit ForceKeep(const std::string& name = "ForceKeep");
	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	
	};


	class ForceDrag :public aris::core::CloneObject<ForceDrag, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~ForceDrag();
		explicit ForceDrag(const std::string& name = "ForceDrag");
	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	
	};


	class PegInHole :public aris::core::CloneObject<PegInHole, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~PegInHole();
		explicit PegInHole(const std::string& name = "PegInHole");
	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;

	};

	class PegInHoleTest :public aris::core::CloneObject<PegInHoleTest, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~PegInHoleTest();
		explicit PegInHoleTest(const std::string& name = "PegInHoleTest");
	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;

	};

	class PegOutHole :public aris::core::CloneObject<PegOutHole, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~PegOutHole();
		explicit PegOutHole(const std::string& name = "PegOutHole");
	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;

	};

	class PegXYZ :public aris::core::CloneObject<PegXYZ, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~PegXYZ();
		explicit PegXYZ(const std::string& name = "PegXYZ");
	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;

	};

	class PegRPY :public aris::core::CloneObject<PegRPY, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~PegRPY();
		explicit PegRPY(const std::string& name = "PegRPY");
	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;

	};


}



#endif
