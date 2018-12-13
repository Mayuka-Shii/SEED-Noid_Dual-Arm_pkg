//DualReturnID.h
#ifndef __DualReturnID_H__
#define __DualReturnID_H__

static JARA_ARM_DUAL::RETURN_ID* D_RETURN_CODE(long id, const char *comment)
{
  JARA_ARM_DUAL::RETURN_ID_var RETURNCODE = new JARA_ARM_DUAL::RETURN_ID;
  RETURNCODE->id = id;
  RETURNCODE->comment = comment;
  return RETURNCODE._retn(); 
}

#define D_RETURNID(id,comment) { return D_RETURN_CODE(id,comment);}
#define D_RETURNID_OK D_RETURNID(JARA_ARM_LEFT::OK,"オペレーションを正常に受け付け.")
#define D_RETURNID_NG D_RETURNID(JARA_ARM_LEFT::NG,"オペレーション拒否.")
#define D_RETURNID_STATUS_ERR D_RETURNID(JARA_ARM_LEFT::STATUS_ERR,"オペレーションを受け付け可能な状態でない.")
#define D_RETURNID_VALUE_ERR D_RETURNID(JARA_ARM_LEFT::VALUE_ERR,"引数が不正.")
#define D_RETURNID_NOT_SV_ON_ERR D_RETURNID(JARA_ARM_LEFT::NOT_SV_ON_ERR,"全ての軸のサーボが入っていない.")
#define D_RETURNID_FULL_MOTION_QUEUE_ERR D_RETURNID(JARA_ARM_LEFT::FULL_MOTION_QUEUE_ERR,"バッファが一杯.")
#define D_RETURNID_NOT_IMPLEMENTED D_RETURNID(JARA_ARM_LEFT::NOT_IMPLEMENTED,"未実装のコマンド.")

#endif//__DualReturnID_H__
