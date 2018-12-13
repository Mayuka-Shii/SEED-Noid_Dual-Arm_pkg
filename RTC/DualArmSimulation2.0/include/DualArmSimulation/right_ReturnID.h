//RightReturnID.h
#ifndef __RightReturnID_H__
#define __RightReturnID_H__

static JARA_ARM::RETURN_ID* R_RETURN_CODE(int id, const char *comment)
{
  JARA_ARM::RETURN_ID_var RETURNCODE = new JARA_ARM::RETURN_ID;
  RETURNCODE->id = id;
  RETURNCODE->comment = comment;
  return RETURNCODE._retn(); 
}

#define R_RETURNID(id,comment) { return R_RETURN_CODE(id,comment);}

#define R_RETURNID_OK R_RETURNID(JARA_ARM::OK,"�I�y���[�V�����𐳏�Ɏ󂯕t��.")
#define R_RETURNID_NG R_RETURNID(JARA_ARM::NG,"�I�y���[�V��������.")
#define RETURNID_STATUS_ERR R_RETURNID(JARA_ARM_LEFT::STATUS_ERR,"�I�y���[�V�������󂯕t���\�ȏ�ԂłȂ�.")
#define R_RETURNID_VALUE_ERR R_RETURNID(JARA_ARM_LEFT::VALUE_ERR,"�������s��.")
#define R_RETURNID_NOT_SV_ON_ERR R_RETURNID(JARA_ARM_LEFT::NOT_SV_ON_ERR,"�S�Ă̎��̃T�[�{�������Ă��Ȃ�.")
#define R_RETURNID_FULL_MOTION_QUEUE_ERR R_RETURNID(JARA_ARM_LEFT::FULL_MOTION_QUEUE_ERR,"�o�b�t�@����t.")
#define R_RETURNID_NOT_IMPLEMENTED R_RETURNID(JARA_ARM_LEFT::NOT_IMPLEMENTED,"�������̃R�}���h.")

#endif//__RightReturnID_H__
