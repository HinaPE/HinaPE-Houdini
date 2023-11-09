#include "SOP01.h"

#include <OP/OP_Operator.h>
#include <OP/OP_OperatorTable.h>
#include <PRM/PRM_Template.h>

#include <UT/UT_DSOVersion.h> // Very Important!!! Must Include This

static auto SOP01Constructor(OP_Network *net, const char *name, OP_Operator *op) -> OP_Node *
{
	return new SOP01(net, name, op);
}

void newSopOperator(OP_OperatorTable *table)
{
	static PRM_Name names[] = {
			PRM_Name("iso",          "Iso Value"),
			PRM_Name("buildpolysoup","Build Polygon Soup"),
	};

	static PRM_Template	 theTemplates[] = {
			PRM_Template(PRM_FLT,    1, &names[0], PRMzeroDefaults),
			PRM_Template(PRM_TOGGLE, 1, &names[1], PRMzeroDefaults),
			PRM_Template()
	};

	auto *op = new OP_Operator(
			"hina_sop01",
			"HinaSOP01",
			SOP01Constructor,
			theTemplates,
			1,
			1,
			0);

	op->setOpTabSubMenuPath("HinaPE");
	table->addOperator(op);
}
