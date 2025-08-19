#ifndef PLAN_H
#define PLAN_H

#include <aris.hpp>

namespace basic_cmd
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
    
	class ModelInit : public aris::core::CloneObject<ModelInit, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~ModelInit();
		explicit ModelInit(const std::string& name = "ModelInit");
    private:
	};

	class TCurveTest :public aris::core::CloneObject<TCurveTest, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~TCurveTest();
		explicit TCurveTest(const std::string& name = "TCurveTest");
	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
}
#endif