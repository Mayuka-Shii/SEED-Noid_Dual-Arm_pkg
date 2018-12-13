//LeftReturnID.h
#ifndef __LeftReturnID_H__
#define __LeftReturnID_H__

static JARA_ARM_LEFT::RETURN_ID* L_RETURN_CODE(int id, const char *comment)
{
  JARA_ARM_LEFT::RETURN_ID_var RETURNCODE = new JARA_ARM_LEFT::RETURN_ID;
  RETURNCODE->id = id;
  RETURNCODE->comment = comment;
  return RETURNCODE._retn(); 
}

#define L_RETURNID(id,comment) { return L_RETURN_CODE(id,comment);}

#define L_RETURNID_OK L_RETURNID(JARA_ARM_LEFT::OK,"オペレーションを正常に受け付け.")
#define L_RETURNID_NG L_RETURNID(JARA_ARM_LEFT::NG,"オペレーション拒否.")
#define L_RETURNID_STATUS_ERR L_RETURNID(JARA_ARM_LEFT::STATUS_ERR,"オペレーションを受け付け可能な状態でない.")
#define L_RETURNID_VALUE_ERR L_RETURNID(JARA_ARM_LEFT::VALUE_ERR,"引数が不正.")
#define L_RETURNID_NOT_SV_ON_ERR L_RETURNID(JARA_ARM_LEFT::NOT_SV_ON_ERR,"全ての軸のサーボが入っていない.")
#define L_RETURNID_FULL_MOTION_QUEUE_ERR L_RETURNID(JARA_ARM_LEFT::FULL_MOTION_QUEUE_ERR,"バッファが一杯.")
#define L_RETURNID_NOT_IMPLEMENTED L_RETURNID(JARA_ARM_LEFT::NOT_IMPLEMENTED,"未実装のコマンド.")

#endif//__LEFTReturnID_H__
