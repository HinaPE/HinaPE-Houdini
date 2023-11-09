#ifndef INC_02_SIM_SIM01_H
#define INC_02_SIM_SIM01_H

#include <SIM/SIM_Data.h>
#include <SIM/SIM_DataUtils.h>
#include <SIM/SIM_OptionsUser.h>

class SIM01 : public SIM_Data, public SIM_OptionsUser
{
public:
	GETSET_DATA_FUNCS_F("test", Test);

protected:
	explicit SIM01(const SIM_DataFactory *factory);
	~SIM01() override;

private:
	static auto getSIM01Description() -> const SIM_DopDescription *;

DECLARE_STANDARD_GETCASTTOTYPE();
DECLARE_DATAFACTORY(SIM01, SIM_Data, "SIM 01", getSIM01Description());
};

#endif //INC_02_SIM_SIM01_H
