#ifndef PLAN_H
#define PLAN_H

#include <aris.hpp>

namespace force_cmd
{
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
}

#endif
