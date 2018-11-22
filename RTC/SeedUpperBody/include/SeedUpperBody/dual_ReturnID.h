//DualReturnID.h
#ifndef __DualReturnID_H__
#define __DualReturnID_H__

static DualManipulatorCommonInterface::RETURN_ID* RETURN_CODE(long id, const char *comment)
{
  DualManipulatorCommonInterface::RETURN_ID_var RETURNCODE = new DualManipulatorCommonInterface::RETURN_ID;
  RETURNCODE->id = id;
  RETURNCODE->comment = comment;
  return RETURNCODE._retn(); 
}

#define RETURNID(id,comment) { return RETURN_CODE(id,comment);}
#define RETURNID_OK RETURNID(0,"ok")
#define RETURNID_NG RETURNID(-1,"ng")

#endif//__DualReturnID_H__
