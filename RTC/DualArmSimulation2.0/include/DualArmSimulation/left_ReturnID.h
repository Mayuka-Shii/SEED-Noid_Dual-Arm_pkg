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

#define L_RETURNID_OK L_RETURNID(JARA_ARM_LEFT::OK,"�I�y���[�V�����𐳏�Ɏ󂯕t��.")
#define L_RETURNID_NG L_RETURNID(JARA_ARM_LEFT::NG,"�I�y���[�V��������.")
#define L_RETURNID_STATUS_ERR L_RETURNID(JARA_ARM_LEFT::STATUS_ERR,"�I�y���[�V�������󂯕t���\�ȏ�ԂłȂ�.")
#define L_RETURNID_VALUE_ERR L_RETURNID(JARA_ARM_LEFT::VALUE_ERR,"�������s��.")
#define L_RETURNID_NOT_SV_ON_ERR L_RETURNID(JARA_ARM_LEFT::NOT_SV_ON_ERR,"�S�Ă̎��̃T�[�{�������Ă��Ȃ�.")
#define L_RETURNID_FULL_MOTION_QUEUE_ERR L_RETURNID(JARA_ARM_LEFT::FULL_MOTION_QUEUE_ERR,"�o�b�t�@����t.")
#define L_RETURNID_NOT_IMPLEMENTED L_RETURNID(JARA_ARM_LEFT::NOT_IMPLEMENTED,"�������̃R�}���h.")

#endif//__LEFTReturnID_H__
