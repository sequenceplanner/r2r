  pub mod msg {
    use super::super::*;

                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Ack {

                              pub cls_id: u8,
pub msg_id: u8,

                          }

                          impl WrappedTypesupport for Ack { 

            type CStruct = ublox_msgs__msg__Ack; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__Ack() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__Ack {

                unsafe { ublox_msgs__msg__Ack__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__Ack) -> () {

                unsafe { ublox_msgs__msg__Ack__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Ack {
  Ack {
cls_id: msg.cls_id,
msg_id: msg.msg_id,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.cls_id = self.cls_id;
msg.msg_id = self.msg_id;
}



        }


                          
                          impl Default for Ack {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Ack>::new();
                                  Ack::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct AidALM {

                              pub svid: u32,
pub week: u32,
pub dwrd: Vec<u32>,

                          }

                          impl WrappedTypesupport for AidALM { 

            type CStruct = ublox_msgs__msg__AidALM; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__AidALM() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__AidALM {

                unsafe { ublox_msgs__msg__AidALM__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__AidALM) -> () {

                unsafe { ublox_msgs__msg__AidALM__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> AidALM {
  AidALM {
svid: msg.svid,
week: msg.week,
// is_upper_bound_: false
// member.array_size_ : 0
dwrd: msg.dwrd.to_vec(),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.svid = self.svid;
msg.week = self.week;
msg.dwrd.update(&self.dwrd);
}



        }


                          
                          impl Default for AidALM {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<AidALM>::new();
                                  AidALM::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct AidEPH {

                              pub svid: u32,
pub how: u32,
pub sf1d: Vec<u32>,
pub sf2d: Vec<u32>,
pub sf3d: Vec<u32>,

                          }

                          impl WrappedTypesupport for AidEPH { 

            type CStruct = ublox_msgs__msg__AidEPH; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__AidEPH() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__AidEPH {

                unsafe { ublox_msgs__msg__AidEPH__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__AidEPH) -> () {

                unsafe { ublox_msgs__msg__AidEPH__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> AidEPH {
  AidEPH {
svid: msg.svid,
how: msg.how,
// is_upper_bound_: false
// member.array_size_ : 0
sf1d: msg.sf1d.to_vec(),
// is_upper_bound_: false
// member.array_size_ : 0
sf2d: msg.sf2d.to_vec(),
// is_upper_bound_: false
// member.array_size_ : 0
sf3d: msg.sf3d.to_vec(),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.svid = self.svid;
msg.how = self.how;
msg.sf1d.update(&self.sf1d);
msg.sf2d.update(&self.sf2d);
msg.sf3d.update(&self.sf3d);
}



        }


                          
                          impl Default for AidEPH {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<AidEPH>::new();
                                  AidEPH::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct AidHUI {

                              pub health: u32,
pub utc_a0: f64,
pub utc_a1: f64,
pub utc_tow: i32,
pub utc_wnt: i16,
pub utc_ls: i16,
pub utc_wnf: i16,
pub utc_dn: i16,
pub utc_lsf: i16,
pub utc_spare: i16,
pub klob_a0: f32,
pub klob_a1: f32,
pub klob_a2: f32,
pub klob_a3: f32,
pub klob_b0: f32,
pub klob_b1: f32,
pub klob_b2: f32,
pub klob_b3: f32,
pub flags: u32,

                          }

                          impl WrappedTypesupport for AidHUI { 

            type CStruct = ublox_msgs__msg__AidHUI; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__AidHUI() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__AidHUI {

                unsafe { ublox_msgs__msg__AidHUI__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__AidHUI) -> () {

                unsafe { ublox_msgs__msg__AidHUI__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> AidHUI {
  AidHUI {
health: msg.health,
utc_a0: msg.utc_a0,
utc_a1: msg.utc_a1,
utc_tow: msg.utc_tow,
utc_wnt: msg.utc_wnt,
utc_ls: msg.utc_ls,
utc_wnf: msg.utc_wnf,
utc_dn: msg.utc_dn,
utc_lsf: msg.utc_lsf,
utc_spare: msg.utc_spare,
klob_a0: msg.klob_a0,
klob_a1: msg.klob_a1,
klob_a2: msg.klob_a2,
klob_a3: msg.klob_a3,
klob_b0: msg.klob_b0,
klob_b1: msg.klob_b1,
klob_b2: msg.klob_b2,
klob_b3: msg.klob_b3,
flags: msg.flags,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.health = self.health;
msg.utc_a0 = self.utc_a0;
msg.utc_a1 = self.utc_a1;
msg.utc_tow = self.utc_tow;
msg.utc_wnt = self.utc_wnt;
msg.utc_ls = self.utc_ls;
msg.utc_wnf = self.utc_wnf;
msg.utc_dn = self.utc_dn;
msg.utc_lsf = self.utc_lsf;
msg.utc_spare = self.utc_spare;
msg.klob_a0 = self.klob_a0;
msg.klob_a1 = self.klob_a1;
msg.klob_a2 = self.klob_a2;
msg.klob_a3 = self.klob_a3;
msg.klob_b0 = self.klob_b0;
msg.klob_b1 = self.klob_b1;
msg.klob_b2 = self.klob_b2;
msg.klob_b3 = self.klob_b3;
msg.flags = self.flags;
}



        }


                          
                          impl Default for AidHUI {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<AidHUI>::new();
                                  AidHUI::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct CfgANT {

                              pub flags: u16,
pub pins: u16,

                          }

                          impl WrappedTypesupport for CfgANT { 

            type CStruct = ublox_msgs__msg__CfgANT; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgANT() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__CfgANT {

                unsafe { ublox_msgs__msg__CfgANT__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__CfgANT) -> () {

                unsafe { ublox_msgs__msg__CfgANT__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> CfgANT {
  CfgANT {
flags: msg.flags,
pins: msg.pins,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.flags = self.flags;
msg.pins = self.pins;
}



        }


                          
                          impl Default for CfgANT {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<CfgANT>::new();
                                  CfgANT::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct CfgCFG {

                              pub clear_mask: u32,
pub save_mask: u32,
pub load_mask: u32,
pub device_mask: u8,

                          }

                          impl WrappedTypesupport for CfgCFG { 

            type CStruct = ublox_msgs__msg__CfgCFG; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgCFG() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__CfgCFG {

                unsafe { ublox_msgs__msg__CfgCFG__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__CfgCFG) -> () {

                unsafe { ublox_msgs__msg__CfgCFG__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> CfgCFG {
  CfgCFG {
clear_mask: msg.clear_mask,
save_mask: msg.save_mask,
load_mask: msg.load_mask,
device_mask: msg.device_mask,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.clear_mask = self.clear_mask;
msg.save_mask = self.save_mask;
msg.load_mask = self.load_mask;
msg.device_mask = self.device_mask;
}



        }


                          
                          impl Default for CfgCFG {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<CfgCFG>::new();
                                  CfgCFG::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct CfgDAT {

                              pub datum_num: u16,
pub datum_name: Vec<u8>,
pub maj_a: f64,
pub flat: f64,
pub d_x: f32,
pub d_y: f32,
pub d_z: f32,
pub rot_x: f32,
pub rot_y: f32,
pub rot_z: f32,
pub scale: f32,

                          }

                          impl WrappedTypesupport for CfgDAT { 

            type CStruct = ublox_msgs__msg__CfgDAT; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgDAT() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__CfgDAT {

                unsafe { ublox_msgs__msg__CfgDAT__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__CfgDAT) -> () {

                unsafe { ublox_msgs__msg__CfgDAT__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> CfgDAT {
  CfgDAT {
datum_num: msg.datum_num,
// is_upper_bound_: false
// member.array_size_ : 6
datum_name: msg.datum_name.to_vec(),
maj_a: msg.maj_a,
flat: msg.flat,
d_x: msg.d_x,
d_y: msg.d_y,
d_z: msg.d_z,
rot_x: msg.rot_x,
rot_y: msg.rot_y,
rot_z: msg.rot_z,
scale: msg.scale,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.datum_num = self.datum_num;
assert_eq!(self.datum_name.len(), 6, "Field {} is fixed size of {}!", "datum_name", 6);
msg.datum_name.copy_from_slice(&self.datum_name[..6]);
msg.maj_a = self.maj_a;
msg.flat = self.flat;
msg.d_x = self.d_x;
msg.d_y = self.d_y;
msg.d_z = self.d_z;
msg.rot_x = self.rot_x;
msg.rot_y = self.rot_y;
msg.rot_z = self.rot_z;
msg.scale = self.scale;
}



        }


                          
                          impl Default for CfgDAT {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<CfgDAT>::new();
                                  CfgDAT::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct CfgDGNSS {

                              pub dgnss_mode: u8,
pub reserved0: Vec<u8>,

                          }

                          impl WrappedTypesupport for CfgDGNSS { 

            type CStruct = ublox_msgs__msg__CfgDGNSS; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgDGNSS() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__CfgDGNSS {

                unsafe { ublox_msgs__msg__CfgDGNSS__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__CfgDGNSS) -> () {

                unsafe { ublox_msgs__msg__CfgDGNSS__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> CfgDGNSS {
  CfgDGNSS {
dgnss_mode: msg.dgnss_mode,
// is_upper_bound_: false
// member.array_size_ : 3
reserved0: msg.reserved0.to_vec(),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.dgnss_mode = self.dgnss_mode;
assert_eq!(self.reserved0.len(), 3, "Field {} is fixed size of {}!", "reserved0", 3);
msg.reserved0.copy_from_slice(&self.reserved0[..3]);
}



        }


                          
                          impl Default for CfgDGNSS {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<CfgDGNSS>::new();
                                  CfgDGNSS::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct CfgGNSS {

                              pub msg_ver: u8,
pub num_trk_ch_hw: u8,
pub num_trk_ch_use: u8,
pub num_config_blocks: u8,
pub blocks: Vec<ublox_msgs::msg::CfgGNSSBlock>,

                          }

                          impl WrappedTypesupport for CfgGNSS { 

            type CStruct = ublox_msgs__msg__CfgGNSS; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgGNSS() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__CfgGNSS {

                unsafe { ublox_msgs__msg__CfgGNSS__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__CfgGNSS) -> () {

                unsafe { ublox_msgs__msg__CfgGNSS__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> CfgGNSS {
  CfgGNSS {
msg_ver: msg.msg_ver,
num_trk_ch_hw: msg.num_trk_ch_hw,
num_trk_ch_use: msg.num_trk_ch_use,
num_config_blocks: msg.num_config_blocks,
// is_upper_bound_: false
// member.array_size_ : 0
blocks : {
let mut temp = Vec::with_capacity(msg.blocks.size);
let slice = unsafe { std::slice::from_raw_parts(msg.blocks.data, msg.blocks.size)};
for s in slice { temp.push(ublox_msgs::msg::CfgGNSSBlock::from_native(s)); }
temp },
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.msg_ver = self.msg_ver;
msg.num_trk_ch_hw = self.num_trk_ch_hw;
msg.num_trk_ch_use = self.num_trk_ch_use;
msg.num_config_blocks = self.num_config_blocks;
unsafe { ublox_msgs__msg__CfgGNSSBlock__Sequence__fini(&mut msg.blocks) };
unsafe { ublox_msgs__msg__CfgGNSSBlock__Sequence__init(&mut msg.blocks, self.blocks.len()) };
let slice = unsafe { std::slice::from_raw_parts_mut(msg.blocks.data, msg.blocks.size)};
for (t,s) in slice.iter_mut().zip(&self.blocks) { s.copy_to_native(t);}
}



        }


                          
                          impl Default for CfgGNSS {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<CfgGNSS>::new();
                                  CfgGNSS::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct CfgGNSSBlock {

                              pub gnss_id: u8,
pub res_trk_ch: u8,
pub max_trk_ch: u8,
pub reserved1: u8,
pub flags: u32,

                          }

                          impl WrappedTypesupport for CfgGNSSBlock { 

            type CStruct = ublox_msgs__msg__CfgGNSSBlock; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgGNSSBlock() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__CfgGNSSBlock {

                unsafe { ublox_msgs__msg__CfgGNSSBlock__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__CfgGNSSBlock) -> () {

                unsafe { ublox_msgs__msg__CfgGNSSBlock__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> CfgGNSSBlock {
  CfgGNSSBlock {
gnss_id: msg.gnss_id,
res_trk_ch: msg.res_trk_ch,
max_trk_ch: msg.max_trk_ch,
reserved1: msg.reserved1,
flags: msg.flags,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.gnss_id = self.gnss_id;
msg.res_trk_ch = self.res_trk_ch;
msg.max_trk_ch = self.max_trk_ch;
msg.reserved1 = self.reserved1;
msg.flags = self.flags;
}



        }


                          
                          impl Default for CfgGNSSBlock {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<CfgGNSSBlock>::new();
                                  CfgGNSSBlock::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct CfgHNR {

                              pub high_nav_rate: u8,
pub reserved0: Vec<u8>,

                          }

                          impl WrappedTypesupport for CfgHNR { 

            type CStruct = ublox_msgs__msg__CfgHNR; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgHNR() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__CfgHNR {

                unsafe { ublox_msgs__msg__CfgHNR__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__CfgHNR) -> () {

                unsafe { ublox_msgs__msg__CfgHNR__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> CfgHNR {
  CfgHNR {
high_nav_rate: msg.high_nav_rate,
// is_upper_bound_: false
// member.array_size_ : 3
reserved0: msg.reserved0.to_vec(),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.high_nav_rate = self.high_nav_rate;
assert_eq!(self.reserved0.len(), 3, "Field {} is fixed size of {}!", "reserved0", 3);
msg.reserved0.copy_from_slice(&self.reserved0[..3]);
}



        }


                          
                          impl Default for CfgHNR {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<CfgHNR>::new();
                                  CfgHNR::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct CfgINF {

                              pub blocks: Vec<ublox_msgs::msg::CfgINFBlock>,

                          }

                          impl WrappedTypesupport for CfgINF { 

            type CStruct = ublox_msgs__msg__CfgINF; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgINF() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__CfgINF {

                unsafe { ublox_msgs__msg__CfgINF__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__CfgINF) -> () {

                unsafe { ublox_msgs__msg__CfgINF__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> CfgINF {
  CfgINF {
// is_upper_bound_: false
// member.array_size_ : 0
blocks : {
let mut temp = Vec::with_capacity(msg.blocks.size);
let slice = unsafe { std::slice::from_raw_parts(msg.blocks.data, msg.blocks.size)};
for s in slice { temp.push(ublox_msgs::msg::CfgINFBlock::from_native(s)); }
temp },
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {unsafe { ublox_msgs__msg__CfgINFBlock__Sequence__fini(&mut msg.blocks) };
unsafe { ublox_msgs__msg__CfgINFBlock__Sequence__init(&mut msg.blocks, self.blocks.len()) };
let slice = unsafe { std::slice::from_raw_parts_mut(msg.blocks.data, msg.blocks.size)};
for (t,s) in slice.iter_mut().zip(&self.blocks) { s.copy_to_native(t);}
}



        }


                          
                          impl Default for CfgINF {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<CfgINF>::new();
                                  CfgINF::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct CfgINFBlock {

                              pub protocol_id: u8,
pub reserved1: Vec<u8>,
pub inf_msg_mask: Vec<u8>,

                          }

                          impl WrappedTypesupport for CfgINFBlock { 

            type CStruct = ublox_msgs__msg__CfgINFBlock; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgINFBlock() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__CfgINFBlock {

                unsafe { ublox_msgs__msg__CfgINFBlock__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__CfgINFBlock) -> () {

                unsafe { ublox_msgs__msg__CfgINFBlock__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> CfgINFBlock {
  CfgINFBlock {
protocol_id: msg.protocol_id,
// is_upper_bound_: false
// member.array_size_ : 3
reserved1: msg.reserved1.to_vec(),
// is_upper_bound_: false
// member.array_size_ : 6
inf_msg_mask: msg.inf_msg_mask.to_vec(),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.protocol_id = self.protocol_id;
assert_eq!(self.reserved1.len(), 3, "Field {} is fixed size of {}!", "reserved1", 3);
msg.reserved1.copy_from_slice(&self.reserved1[..3]);
assert_eq!(self.inf_msg_mask.len(), 6, "Field {} is fixed size of {}!", "inf_msg_mask", 6);
msg.inf_msg_mask.copy_from_slice(&self.inf_msg_mask[..6]);
}



        }


                          
                          impl Default for CfgINFBlock {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<CfgINFBlock>::new();
                                  CfgINFBlock::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct CfgMSG {

                              pub msg_class: u8,
pub msg_id: u8,
pub rate: u8,

                          }

                          impl WrappedTypesupport for CfgMSG { 

            type CStruct = ublox_msgs__msg__CfgMSG; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgMSG() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__CfgMSG {

                unsafe { ublox_msgs__msg__CfgMSG__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__CfgMSG) -> () {

                unsafe { ublox_msgs__msg__CfgMSG__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> CfgMSG {
  CfgMSG {
msg_class: msg.msg_class,
msg_id: msg.msg_id,
rate: msg.rate,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.msg_class = self.msg_class;
msg.msg_id = self.msg_id;
msg.rate = self.rate;
}



        }


                          
                          impl Default for CfgMSG {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<CfgMSG>::new();
                                  CfgMSG::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct CfgNAV5 {

                              pub mask: u16,
pub dyn_model: u8,
pub fix_mode: u8,
pub fixed_alt: i32,
pub fixed_alt_var: u32,
pub min_elev: i8,
pub dr_limit: u8,
pub p_dop: u16,
pub t_dop: u16,
pub p_acc: u16,
pub t_acc: u16,
pub static_hold_thresh: u8,
pub dgnss_time_out: u8,
pub cno_thresh_num_svs: u8,
pub cno_thresh: u8,
pub reserved1: Vec<u8>,
pub static_hold_max_dist: u16,
pub utc_standard: u8,
pub reserved2: Vec<u8>,

                          }

                          impl WrappedTypesupport for CfgNAV5 { 

            type CStruct = ublox_msgs__msg__CfgNAV5; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgNAV5() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__CfgNAV5 {

                unsafe { ublox_msgs__msg__CfgNAV5__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__CfgNAV5) -> () {

                unsafe { ublox_msgs__msg__CfgNAV5__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> CfgNAV5 {
  CfgNAV5 {
mask: msg.mask,
dyn_model: msg.dyn_model,
fix_mode: msg.fix_mode,
fixed_alt: msg.fixed_alt,
fixed_alt_var: msg.fixed_alt_var,
min_elev: msg.min_elev,
dr_limit: msg.dr_limit,
p_dop: msg.p_dop,
t_dop: msg.t_dop,
p_acc: msg.p_acc,
t_acc: msg.t_acc,
static_hold_thresh: msg.static_hold_thresh,
dgnss_time_out: msg.dgnss_time_out,
cno_thresh_num_svs: msg.cno_thresh_num_svs,
cno_thresh: msg.cno_thresh,
// is_upper_bound_: false
// member.array_size_ : 2
reserved1: msg.reserved1.to_vec(),
static_hold_max_dist: msg.static_hold_max_dist,
utc_standard: msg.utc_standard,
// is_upper_bound_: false
// member.array_size_ : 5
reserved2: msg.reserved2.to_vec(),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.mask = self.mask;
msg.dyn_model = self.dyn_model;
msg.fix_mode = self.fix_mode;
msg.fixed_alt = self.fixed_alt;
msg.fixed_alt_var = self.fixed_alt_var;
msg.min_elev = self.min_elev;
msg.dr_limit = self.dr_limit;
msg.p_dop = self.p_dop;
msg.t_dop = self.t_dop;
msg.p_acc = self.p_acc;
msg.t_acc = self.t_acc;
msg.static_hold_thresh = self.static_hold_thresh;
msg.dgnss_time_out = self.dgnss_time_out;
msg.cno_thresh_num_svs = self.cno_thresh_num_svs;
msg.cno_thresh = self.cno_thresh;
assert_eq!(self.reserved1.len(), 2, "Field {} is fixed size of {}!", "reserved1", 2);
msg.reserved1.copy_from_slice(&self.reserved1[..2]);
msg.static_hold_max_dist = self.static_hold_max_dist;
msg.utc_standard = self.utc_standard;
assert_eq!(self.reserved2.len(), 5, "Field {} is fixed size of {}!", "reserved2", 5);
msg.reserved2.copy_from_slice(&self.reserved2[..5]);
}



        }


                          
                          impl Default for CfgNAV5 {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<CfgNAV5>::new();
                                  CfgNAV5::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct CfgNAVX5 {

                              pub version: u16,
pub mask1: u16,
pub mask2: u32,
pub reserved1: Vec<u8>,
pub min_svs: u8,
pub max_svs: u8,
pub min_cno: u8,
pub reserved2: u8,
pub ini_fix3d: u8,
pub reserved3: Vec<u8>,
pub ack_aiding: u8,
pub wkn_rollover: u16,
pub sig_atten_comp_mode: u8,
pub reserved4: Vec<u8>,
pub use_ppp: u8,
pub aop_cfg: u8,
pub reserved5: Vec<u8>,
pub aop_orb_max_err: u16,
pub reserved6: Vec<u8>,
pub use_adr: u8,

                          }

                          impl WrappedTypesupport for CfgNAVX5 { 

            type CStruct = ublox_msgs__msg__CfgNAVX5; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgNAVX5() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__CfgNAVX5 {

                unsafe { ublox_msgs__msg__CfgNAVX5__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__CfgNAVX5) -> () {

                unsafe { ublox_msgs__msg__CfgNAVX5__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> CfgNAVX5 {
  CfgNAVX5 {
version: msg.version,
mask1: msg.mask1,
mask2: msg.mask2,
// is_upper_bound_: false
// member.array_size_ : 2
reserved1: msg.reserved1.to_vec(),
min_svs: msg.min_svs,
max_svs: msg.max_svs,
min_cno: msg.min_cno,
reserved2: msg.reserved2,
ini_fix3d: msg.ini_fix3d,
// is_upper_bound_: false
// member.array_size_ : 2
reserved3: msg.reserved3.to_vec(),
ack_aiding: msg.ack_aiding,
wkn_rollover: msg.wkn_rollover,
sig_atten_comp_mode: msg.sig_atten_comp_mode,
// is_upper_bound_: false
// member.array_size_ : 5
reserved4: msg.reserved4.to_vec(),
use_ppp: msg.use_ppp,
aop_cfg: msg.aop_cfg,
// is_upper_bound_: false
// member.array_size_ : 2
reserved5: msg.reserved5.to_vec(),
aop_orb_max_err: msg.aop_orb_max_err,
// is_upper_bound_: false
// member.array_size_ : 7
reserved6: msg.reserved6.to_vec(),
use_adr: msg.use_adr,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.version = self.version;
msg.mask1 = self.mask1;
msg.mask2 = self.mask2;
assert_eq!(self.reserved1.len(), 2, "Field {} is fixed size of {}!", "reserved1", 2);
msg.reserved1.copy_from_slice(&self.reserved1[..2]);
msg.min_svs = self.min_svs;
msg.max_svs = self.max_svs;
msg.min_cno = self.min_cno;
msg.reserved2 = self.reserved2;
msg.ini_fix3d = self.ini_fix3d;
assert_eq!(self.reserved3.len(), 2, "Field {} is fixed size of {}!", "reserved3", 2);
msg.reserved3.copy_from_slice(&self.reserved3[..2]);
msg.ack_aiding = self.ack_aiding;
msg.wkn_rollover = self.wkn_rollover;
msg.sig_atten_comp_mode = self.sig_atten_comp_mode;
assert_eq!(self.reserved4.len(), 5, "Field {} is fixed size of {}!", "reserved4", 5);
msg.reserved4.copy_from_slice(&self.reserved4[..5]);
msg.use_ppp = self.use_ppp;
msg.aop_cfg = self.aop_cfg;
assert_eq!(self.reserved5.len(), 2, "Field {} is fixed size of {}!", "reserved5", 2);
msg.reserved5.copy_from_slice(&self.reserved5[..2]);
msg.aop_orb_max_err = self.aop_orb_max_err;
assert_eq!(self.reserved6.len(), 7, "Field {} is fixed size of {}!", "reserved6", 7);
msg.reserved6.copy_from_slice(&self.reserved6[..7]);
msg.use_adr = self.use_adr;
}



        }


                          
                          impl Default for CfgNAVX5 {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<CfgNAVX5>::new();
                                  CfgNAVX5::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct CfgNMEA {

                              pub filter: u8,
pub nmea_version: u8,
pub num_sv: u8,
pub flags: u8,
pub gnss_to_filter: u32,
pub sv_numbering: u8,
pub main_talker_id: u8,
pub gsv_talker_id: u8,
pub version: u8,
pub bds_talker_id: Vec<u8>,
pub reserved1: Vec<u8>,

                          }

                          impl WrappedTypesupport for CfgNMEA { 

            type CStruct = ublox_msgs__msg__CfgNMEA; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgNMEA() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__CfgNMEA {

                unsafe { ublox_msgs__msg__CfgNMEA__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__CfgNMEA) -> () {

                unsafe { ublox_msgs__msg__CfgNMEA__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> CfgNMEA {
  CfgNMEA {
filter: msg.filter,
nmea_version: msg.nmea_version,
num_sv: msg.num_sv,
flags: msg.flags,
gnss_to_filter: msg.gnss_to_filter,
sv_numbering: msg.sv_numbering,
main_talker_id: msg.main_talker_id,
gsv_talker_id: msg.gsv_talker_id,
version: msg.version,
// is_upper_bound_: false
// member.array_size_ : 2
bds_talker_id: msg.bds_talker_id.to_vec(),
// is_upper_bound_: false
// member.array_size_ : 6
reserved1: msg.reserved1.to_vec(),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.filter = self.filter;
msg.nmea_version = self.nmea_version;
msg.num_sv = self.num_sv;
msg.flags = self.flags;
msg.gnss_to_filter = self.gnss_to_filter;
msg.sv_numbering = self.sv_numbering;
msg.main_talker_id = self.main_talker_id;
msg.gsv_talker_id = self.gsv_talker_id;
msg.version = self.version;
assert_eq!(self.bds_talker_id.len(), 2, "Field {} is fixed size of {}!", "bds_talker_id", 2);
msg.bds_talker_id.copy_from_slice(&self.bds_talker_id[..2]);
assert_eq!(self.reserved1.len(), 6, "Field {} is fixed size of {}!", "reserved1", 6);
msg.reserved1.copy_from_slice(&self.reserved1[..6]);
}



        }


                          
                          impl Default for CfgNMEA {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<CfgNMEA>::new();
                                  CfgNMEA::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct CfgNMEA6 {

                              pub filter: u8,
pub version: u8,
pub num_sv: u8,
pub flags: u8,

                          }

                          impl WrappedTypesupport for CfgNMEA6 { 

            type CStruct = ublox_msgs__msg__CfgNMEA6; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgNMEA6() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__CfgNMEA6 {

                unsafe { ublox_msgs__msg__CfgNMEA6__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__CfgNMEA6) -> () {

                unsafe { ublox_msgs__msg__CfgNMEA6__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> CfgNMEA6 {
  CfgNMEA6 {
filter: msg.filter,
version: msg.version,
num_sv: msg.num_sv,
flags: msg.flags,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.filter = self.filter;
msg.version = self.version;
msg.num_sv = self.num_sv;
msg.flags = self.flags;
}



        }


                          
                          impl Default for CfgNMEA6 {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<CfgNMEA6>::new();
                                  CfgNMEA6::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct CfgNMEA7 {

                              pub filter: u8,
pub nmea_version: u8,
pub num_sv: u8,
pub flags: u8,
pub gnss_to_filter: u32,
pub sv_numbering: u8,
pub main_talker_id: u8,
pub gsv_talker_id: u8,
pub reserved: u8,

                          }

                          impl WrappedTypesupport for CfgNMEA7 { 

            type CStruct = ublox_msgs__msg__CfgNMEA7; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgNMEA7() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__CfgNMEA7 {

                unsafe { ublox_msgs__msg__CfgNMEA7__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__CfgNMEA7) -> () {

                unsafe { ublox_msgs__msg__CfgNMEA7__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> CfgNMEA7 {
  CfgNMEA7 {
filter: msg.filter,
nmea_version: msg.nmea_version,
num_sv: msg.num_sv,
flags: msg.flags,
gnss_to_filter: msg.gnss_to_filter,
sv_numbering: msg.sv_numbering,
main_talker_id: msg.main_talker_id,
gsv_talker_id: msg.gsv_talker_id,
reserved: msg.reserved,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.filter = self.filter;
msg.nmea_version = self.nmea_version;
msg.num_sv = self.num_sv;
msg.flags = self.flags;
msg.gnss_to_filter = self.gnss_to_filter;
msg.sv_numbering = self.sv_numbering;
msg.main_talker_id = self.main_talker_id;
msg.gsv_talker_id = self.gsv_talker_id;
msg.reserved = self.reserved;
}



        }


                          
                          impl Default for CfgNMEA7 {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<CfgNMEA7>::new();
                                  CfgNMEA7::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct CfgPRT {

                              pub port_id: u8,
pub reserved0: u8,
pub tx_ready: u16,
pub mode: u32,
pub baud_rate: u32,
pub in_proto_mask: u16,
pub out_proto_mask: u16,
pub flags: u16,
pub reserved1: u16,

                          }

                          impl WrappedTypesupport for CfgPRT { 

            type CStruct = ublox_msgs__msg__CfgPRT; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgPRT() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__CfgPRT {

                unsafe { ublox_msgs__msg__CfgPRT__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__CfgPRT) -> () {

                unsafe { ublox_msgs__msg__CfgPRT__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> CfgPRT {
  CfgPRT {
port_id: msg.port_id,
reserved0: msg.reserved0,
tx_ready: msg.tx_ready,
mode: msg.mode,
baud_rate: msg.baud_rate,
in_proto_mask: msg.in_proto_mask,
out_proto_mask: msg.out_proto_mask,
flags: msg.flags,
reserved1: msg.reserved1,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.port_id = self.port_id;
msg.reserved0 = self.reserved0;
msg.tx_ready = self.tx_ready;
msg.mode = self.mode;
msg.baud_rate = self.baud_rate;
msg.in_proto_mask = self.in_proto_mask;
msg.out_proto_mask = self.out_proto_mask;
msg.flags = self.flags;
msg.reserved1 = self.reserved1;
}



        }


                          
                          impl Default for CfgPRT {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<CfgPRT>::new();
                                  CfgPRT::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct CfgRATE {

                              pub meas_rate: u16,
pub nav_rate: u16,
pub time_ref: u16,

                          }

                          impl WrappedTypesupport for CfgRATE { 

            type CStruct = ublox_msgs__msg__CfgRATE; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgRATE() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__CfgRATE {

                unsafe { ublox_msgs__msg__CfgRATE__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__CfgRATE) -> () {

                unsafe { ublox_msgs__msg__CfgRATE__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> CfgRATE {
  CfgRATE {
meas_rate: msg.meas_rate,
nav_rate: msg.nav_rate,
time_ref: msg.time_ref,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.meas_rate = self.meas_rate;
msg.nav_rate = self.nav_rate;
msg.time_ref = self.time_ref;
}



        }


                          
                          impl Default for CfgRATE {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<CfgRATE>::new();
                                  CfgRATE::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct CfgRST {

                              pub nav_bbr_mask: u16,
pub reset_mode: u8,
pub reserved1: u8,

                          }

                          impl WrappedTypesupport for CfgRST { 

            type CStruct = ublox_msgs__msg__CfgRST; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgRST() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__CfgRST {

                unsafe { ublox_msgs__msg__CfgRST__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__CfgRST) -> () {

                unsafe { ublox_msgs__msg__CfgRST__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> CfgRST {
  CfgRST {
nav_bbr_mask: msg.nav_bbr_mask,
reset_mode: msg.reset_mode,
reserved1: msg.reserved1,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.nav_bbr_mask = self.nav_bbr_mask;
msg.reset_mode = self.reset_mode;
msg.reserved1 = self.reserved1;
}



        }


                          
                          impl Default for CfgRST {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<CfgRST>::new();
                                  CfgRST::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct CfgSBAS {

                              pub mode: u8,
pub usage: u8,
pub max_sbas: u8,
pub scanmode2: u8,
pub scanmode1: u32,

                          }

                          impl WrappedTypesupport for CfgSBAS { 

            type CStruct = ublox_msgs__msg__CfgSBAS; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgSBAS() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__CfgSBAS {

                unsafe { ublox_msgs__msg__CfgSBAS__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__CfgSBAS) -> () {

                unsafe { ublox_msgs__msg__CfgSBAS__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> CfgSBAS {
  CfgSBAS {
mode: msg.mode,
usage: msg.usage,
max_sbas: msg.max_sbas,
scanmode2: msg.scanmode2,
scanmode1: msg.scanmode1,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.mode = self.mode;
msg.usage = self.usage;
msg.max_sbas = self.max_sbas;
msg.scanmode2 = self.scanmode2;
msg.scanmode1 = self.scanmode1;
}



        }


                          
                          impl Default for CfgSBAS {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<CfgSBAS>::new();
                                  CfgSBAS::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct CfgTMODE3 {

                              pub version: u8,
pub reserved1: u8,
pub flags: u16,
pub ecef_x_or_lat: i32,
pub ecef_y_or_lon: i32,
pub ecef_z_or_alt: i32,
pub ecef_x_or_lat_hp: i8,
pub ecef_y_or_lon_hp: i8,
pub ecef_z_or_alt_hp: i8,
pub reserved2: u8,
pub fixed_pos_acc: u32,
pub svin_min_dur: u32,
pub svin_acc_limit: u32,
pub reserved3: Vec<u8>,

                          }

                          impl WrappedTypesupport for CfgTMODE3 { 

            type CStruct = ublox_msgs__msg__CfgTMODE3; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgTMODE3() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__CfgTMODE3 {

                unsafe { ublox_msgs__msg__CfgTMODE3__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__CfgTMODE3) -> () {

                unsafe { ublox_msgs__msg__CfgTMODE3__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> CfgTMODE3 {
  CfgTMODE3 {
version: msg.version,
reserved1: msg.reserved1,
flags: msg.flags,
ecef_x_or_lat: msg.ecef_x_or_lat,
ecef_y_or_lon: msg.ecef_y_or_lon,
ecef_z_or_alt: msg.ecef_z_or_alt,
ecef_x_or_lat_hp: msg.ecef_x_or_lat_hp,
ecef_y_or_lon_hp: msg.ecef_y_or_lon_hp,
ecef_z_or_alt_hp: msg.ecef_z_or_alt_hp,
reserved2: msg.reserved2,
fixed_pos_acc: msg.fixed_pos_acc,
svin_min_dur: msg.svin_min_dur,
svin_acc_limit: msg.svin_acc_limit,
// is_upper_bound_: false
// member.array_size_ : 8
reserved3: msg.reserved3.to_vec(),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.version = self.version;
msg.reserved1 = self.reserved1;
msg.flags = self.flags;
msg.ecef_x_or_lat = self.ecef_x_or_lat;
msg.ecef_y_or_lon = self.ecef_y_or_lon;
msg.ecef_z_or_alt = self.ecef_z_or_alt;
msg.ecef_x_or_lat_hp = self.ecef_x_or_lat_hp;
msg.ecef_y_or_lon_hp = self.ecef_y_or_lon_hp;
msg.ecef_z_or_alt_hp = self.ecef_z_or_alt_hp;
msg.reserved2 = self.reserved2;
msg.fixed_pos_acc = self.fixed_pos_acc;
msg.svin_min_dur = self.svin_min_dur;
msg.svin_acc_limit = self.svin_acc_limit;
assert_eq!(self.reserved3.len(), 8, "Field {} is fixed size of {}!", "reserved3", 8);
msg.reserved3.copy_from_slice(&self.reserved3[..8]);
}



        }


                          
                          impl Default for CfgTMODE3 {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<CfgTMODE3>::new();
                                  CfgTMODE3::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct CfgUSB {

                              pub vendor_id: u16,
pub product_id: u16,
pub reserved1: Vec<u8>,
pub reserved2: Vec<u8>,
pub power_consumption: u16,
pub flags: u16,
pub vendor_string: Vec<i8>,
pub product_string: Vec<i8>,
pub serial_number: Vec<i8>,

                          }

                          impl WrappedTypesupport for CfgUSB { 

            type CStruct = ublox_msgs__msg__CfgUSB; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgUSB() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__CfgUSB {

                unsafe { ublox_msgs__msg__CfgUSB__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__CfgUSB) -> () {

                unsafe { ublox_msgs__msg__CfgUSB__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> CfgUSB {
  CfgUSB {
vendor_id: msg.vendor_id,
product_id: msg.product_id,
// is_upper_bound_: false
// member.array_size_ : 2
reserved1: msg.reserved1.to_vec(),
// is_upper_bound_: false
// member.array_size_ : 2
reserved2: msg.reserved2.to_vec(),
power_consumption: msg.power_consumption,
flags: msg.flags,
// is_upper_bound_: false
// member.array_size_ : 32
vendor_string: msg.vendor_string.to_vec(),
// is_upper_bound_: false
// member.array_size_ : 32
product_string: msg.product_string.to_vec(),
// is_upper_bound_: false
// member.array_size_ : 32
serial_number: msg.serial_number.to_vec(),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.vendor_id = self.vendor_id;
msg.product_id = self.product_id;
assert_eq!(self.reserved1.len(), 2, "Field {} is fixed size of {}!", "reserved1", 2);
msg.reserved1.copy_from_slice(&self.reserved1[..2]);
assert_eq!(self.reserved2.len(), 2, "Field {} is fixed size of {}!", "reserved2", 2);
msg.reserved2.copy_from_slice(&self.reserved2[..2]);
msg.power_consumption = self.power_consumption;
msg.flags = self.flags;
assert_eq!(self.vendor_string.len(), 32, "Field {} is fixed size of {}!", "vendor_string", 32);
msg.vendor_string.copy_from_slice(&self.vendor_string[..32]);
assert_eq!(self.product_string.len(), 32, "Field {} is fixed size of {}!", "product_string", 32);
msg.product_string.copy_from_slice(&self.product_string[..32]);
assert_eq!(self.serial_number.len(), 32, "Field {} is fixed size of {}!", "serial_number", 32);
msg.serial_number.copy_from_slice(&self.serial_number[..32]);
}



        }


                          
                          impl Default for CfgUSB {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<CfgUSB>::new();
                                  CfgUSB::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct EsfINS {

                              pub bitfield0: u32,
pub reserved1: Vec<u8>,
pub i_tow: u32,
pub x_ang_rate: i32,
pub y_ang_rate: i32,
pub z_ang_rate: i32,
pub x_accel: i32,
pub y_accel: i32,
pub z_accel: i32,

                          }

                          impl WrappedTypesupport for EsfINS { 

            type CStruct = ublox_msgs__msg__EsfINS; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__EsfINS() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__EsfINS {

                unsafe { ublox_msgs__msg__EsfINS__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__EsfINS) -> () {

                unsafe { ublox_msgs__msg__EsfINS__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> EsfINS {
  EsfINS {
bitfield0: msg.bitfield0,
// is_upper_bound_: false
// member.array_size_ : 4
reserved1: msg.reserved1.to_vec(),
i_tow: msg.i_tow,
x_ang_rate: msg.x_ang_rate,
y_ang_rate: msg.y_ang_rate,
z_ang_rate: msg.z_ang_rate,
x_accel: msg.x_accel,
y_accel: msg.y_accel,
z_accel: msg.z_accel,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.bitfield0 = self.bitfield0;
assert_eq!(self.reserved1.len(), 4, "Field {} is fixed size of {}!", "reserved1", 4);
msg.reserved1.copy_from_slice(&self.reserved1[..4]);
msg.i_tow = self.i_tow;
msg.x_ang_rate = self.x_ang_rate;
msg.y_ang_rate = self.y_ang_rate;
msg.z_ang_rate = self.z_ang_rate;
msg.x_accel = self.x_accel;
msg.y_accel = self.y_accel;
msg.z_accel = self.z_accel;
}



        }


                          
                          impl Default for EsfINS {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<EsfINS>::new();
                                  EsfINS::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct EsfMEAS {

                              pub time_tag: u32,
pub flags: u16,
pub id: u16,
pub data: Vec<u32>,
pub calib_t_tag: Vec<u32>,

                          }

                          impl WrappedTypesupport for EsfMEAS { 

            type CStruct = ublox_msgs__msg__EsfMEAS; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__EsfMEAS() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__EsfMEAS {

                unsafe { ublox_msgs__msg__EsfMEAS__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__EsfMEAS) -> () {

                unsafe { ublox_msgs__msg__EsfMEAS__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> EsfMEAS {
  EsfMEAS {
time_tag: msg.time_tag,
flags: msg.flags,
id: msg.id,
// is_upper_bound_: false
// member.array_size_ : 0
data: msg.data.to_vec(),
// is_upper_bound_: false
// member.array_size_ : 0
calib_t_tag: msg.calib_t_tag.to_vec(),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.time_tag = self.time_tag;
msg.flags = self.flags;
msg.id = self.id;
msg.data.update(&self.data);
msg.calib_t_tag.update(&self.calib_t_tag);
}



        }


                          
                          impl Default for EsfMEAS {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<EsfMEAS>::new();
                                  EsfMEAS::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct EsfRAW {

                              pub reserved0: Vec<u8>,
pub blocks: Vec<ublox_msgs::msg::EsfRAWBlock>,

                          }

                          impl WrappedTypesupport for EsfRAW { 

            type CStruct = ublox_msgs__msg__EsfRAW; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__EsfRAW() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__EsfRAW {

                unsafe { ublox_msgs__msg__EsfRAW__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__EsfRAW) -> () {

                unsafe { ublox_msgs__msg__EsfRAW__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> EsfRAW {
  EsfRAW {
// is_upper_bound_: false
// member.array_size_ : 4
reserved0: msg.reserved0.to_vec(),
// is_upper_bound_: false
// member.array_size_ : 0
blocks : {
let mut temp = Vec::with_capacity(msg.blocks.size);
let slice = unsafe { std::slice::from_raw_parts(msg.blocks.data, msg.blocks.size)};
for s in slice { temp.push(ublox_msgs::msg::EsfRAWBlock::from_native(s)); }
temp },
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {assert_eq!(self.reserved0.len(), 4, "Field {} is fixed size of {}!", "reserved0", 4);
msg.reserved0.copy_from_slice(&self.reserved0[..4]);
unsafe { ublox_msgs__msg__EsfRAWBlock__Sequence__fini(&mut msg.blocks) };
unsafe { ublox_msgs__msg__EsfRAWBlock__Sequence__init(&mut msg.blocks, self.blocks.len()) };
let slice = unsafe { std::slice::from_raw_parts_mut(msg.blocks.data, msg.blocks.size)};
for (t,s) in slice.iter_mut().zip(&self.blocks) { s.copy_to_native(t);}
}



        }


                          
                          impl Default for EsfRAW {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<EsfRAW>::new();
                                  EsfRAW::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct EsfRAWBlock {

                              pub data: u32,
pub s_t_tag: u32,

                          }

                          impl WrappedTypesupport for EsfRAWBlock { 

            type CStruct = ublox_msgs__msg__EsfRAWBlock; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__EsfRAWBlock() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__EsfRAWBlock {

                unsafe { ublox_msgs__msg__EsfRAWBlock__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__EsfRAWBlock) -> () {

                unsafe { ublox_msgs__msg__EsfRAWBlock__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> EsfRAWBlock {
  EsfRAWBlock {
data: msg.data,
s_t_tag: msg.s_t_tag,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.data = self.data;
msg.s_t_tag = self.s_t_tag;
}



        }


                          
                          impl Default for EsfRAWBlock {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<EsfRAWBlock>::new();
                                  EsfRAWBlock::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct EsfSTATUS {

                              pub i_tow: u32,
pub version: u8,
pub reserved1: Vec<u8>,
pub fusion_mode: u8,
pub reserved2: Vec<u8>,
pub num_sens: u8,
pub sens: Vec<ublox_msgs::msg::EsfSTATUSSens>,

                          }

                          impl WrappedTypesupport for EsfSTATUS { 

            type CStruct = ublox_msgs__msg__EsfSTATUS; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__EsfSTATUS() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__EsfSTATUS {

                unsafe { ublox_msgs__msg__EsfSTATUS__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__EsfSTATUS) -> () {

                unsafe { ublox_msgs__msg__EsfSTATUS__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> EsfSTATUS {
  EsfSTATUS {
i_tow: msg.i_tow,
version: msg.version,
// is_upper_bound_: false
// member.array_size_ : 7
reserved1: msg.reserved1.to_vec(),
fusion_mode: msg.fusion_mode,
// is_upper_bound_: false
// member.array_size_ : 2
reserved2: msg.reserved2.to_vec(),
num_sens: msg.num_sens,
// is_upper_bound_: false
// member.array_size_ : 0
sens : {
let mut temp = Vec::with_capacity(msg.sens.size);
let slice = unsafe { std::slice::from_raw_parts(msg.sens.data, msg.sens.size)};
for s in slice { temp.push(ublox_msgs::msg::EsfSTATUSSens::from_native(s)); }
temp },
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.i_tow = self.i_tow;
msg.version = self.version;
assert_eq!(self.reserved1.len(), 7, "Field {} is fixed size of {}!", "reserved1", 7);
msg.reserved1.copy_from_slice(&self.reserved1[..7]);
msg.fusion_mode = self.fusion_mode;
assert_eq!(self.reserved2.len(), 2, "Field {} is fixed size of {}!", "reserved2", 2);
msg.reserved2.copy_from_slice(&self.reserved2[..2]);
msg.num_sens = self.num_sens;
unsafe { ublox_msgs__msg__EsfSTATUSSens__Sequence__fini(&mut msg.sens) };
unsafe { ublox_msgs__msg__EsfSTATUSSens__Sequence__init(&mut msg.sens, self.sens.len()) };
let slice = unsafe { std::slice::from_raw_parts_mut(msg.sens.data, msg.sens.size)};
for (t,s) in slice.iter_mut().zip(&self.sens) { s.copy_to_native(t);}
}



        }


                          
                          impl Default for EsfSTATUS {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<EsfSTATUS>::new();
                                  EsfSTATUS::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct EsfSTATUSSens {

                              pub sens_status1: u8,
pub sens_status2: u8,
pub freq: u8,
pub faults: u8,

                          }

                          impl WrappedTypesupport for EsfSTATUSSens { 

            type CStruct = ublox_msgs__msg__EsfSTATUSSens; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__EsfSTATUSSens() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__EsfSTATUSSens {

                unsafe { ublox_msgs__msg__EsfSTATUSSens__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__EsfSTATUSSens) -> () {

                unsafe { ublox_msgs__msg__EsfSTATUSSens__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> EsfSTATUSSens {
  EsfSTATUSSens {
sens_status1: msg.sens_status1,
sens_status2: msg.sens_status2,
freq: msg.freq,
faults: msg.faults,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.sens_status1 = self.sens_status1;
msg.sens_status2 = self.sens_status2;
msg.freq = self.freq;
msg.faults = self.faults;
}



        }


                          
                          impl Default for EsfSTATUSSens {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<EsfSTATUSSens>::new();
                                  EsfSTATUSSens::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct HnrPVT {

                              pub i_tow: u32,
pub year: u16,
pub month: u8,
pub day: u8,
pub hour: u8,
pub min: u8,
pub sec: u8,
pub valid: u8,
pub nano: i32,
pub gps_fix: u8,
pub flags: u8,
pub reserved0: Vec<u8>,
pub lon: i32,
pub lat: i32,
pub height: i32,
pub h_msl: i32,
pub g_speed: i32,
pub speed: i32,
pub head_mot: i32,
pub head_veh: i32,
pub h_acc: u32,
pub v_acc: u32,
pub s_acc: u32,
pub head_acc: u32,
pub reserved1: Vec<u8>,

                          }

                          impl WrappedTypesupport for HnrPVT { 

            type CStruct = ublox_msgs__msg__HnrPVT; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__HnrPVT() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__HnrPVT {

                unsafe { ublox_msgs__msg__HnrPVT__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__HnrPVT) -> () {

                unsafe { ublox_msgs__msg__HnrPVT__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> HnrPVT {
  HnrPVT {
i_tow: msg.i_tow,
year: msg.year,
month: msg.month,
day: msg.day,
hour: msg.hour,
min: msg.min,
sec: msg.sec,
valid: msg.valid,
nano: msg.nano,
gps_fix: msg.gps_fix,
flags: msg.flags,
// is_upper_bound_: false
// member.array_size_ : 2
reserved0: msg.reserved0.to_vec(),
lon: msg.lon,
lat: msg.lat,
height: msg.height,
h_msl: msg.h_msl,
g_speed: msg.g_speed,
speed: msg.speed,
head_mot: msg.head_mot,
head_veh: msg.head_veh,
h_acc: msg.h_acc,
v_acc: msg.v_acc,
s_acc: msg.s_acc,
head_acc: msg.head_acc,
// is_upper_bound_: false
// member.array_size_ : 4
reserved1: msg.reserved1.to_vec(),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.i_tow = self.i_tow;
msg.year = self.year;
msg.month = self.month;
msg.day = self.day;
msg.hour = self.hour;
msg.min = self.min;
msg.sec = self.sec;
msg.valid = self.valid;
msg.nano = self.nano;
msg.gps_fix = self.gps_fix;
msg.flags = self.flags;
assert_eq!(self.reserved0.len(), 2, "Field {} is fixed size of {}!", "reserved0", 2);
msg.reserved0.copy_from_slice(&self.reserved0[..2]);
msg.lon = self.lon;
msg.lat = self.lat;
msg.height = self.height;
msg.h_msl = self.h_msl;
msg.g_speed = self.g_speed;
msg.speed = self.speed;
msg.head_mot = self.head_mot;
msg.head_veh = self.head_veh;
msg.h_acc = self.h_acc;
msg.v_acc = self.v_acc;
msg.s_acc = self.s_acc;
msg.head_acc = self.head_acc;
assert_eq!(self.reserved1.len(), 4, "Field {} is fixed size of {}!", "reserved1", 4);
msg.reserved1.copy_from_slice(&self.reserved1[..4]);
}



        }


                          
                          impl Default for HnrPVT {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<HnrPVT>::new();
                                  HnrPVT::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Inf {

                              pub str_: Vec<u8>,

                          }

                          impl WrappedTypesupport for Inf { 

            type CStruct = ublox_msgs__msg__Inf; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__Inf() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__Inf {

                unsafe { ublox_msgs__msg__Inf__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__Inf) -> () {

                unsafe { ublox_msgs__msg__Inf__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Inf {
  Inf {
// is_upper_bound_: false
// member.array_size_ : 0
str_: msg.str_.to_vec(),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.str_.update(&self.str_);
}



        }


                          
                          impl Default for Inf {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Inf>::new();
                                  Inf::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct MgaGAL {

                              pub type_: u8,
pub version: u8,
pub svid: u8,
pub reserved0: u8,
pub iod_nav: u16,
pub delta_n: i16,
pub m0: i32,
pub e: u32,
pub sqrt_a: u32,
pub omega0: i32,
pub i0: i32,
pub omega: i32,
pub omega_dot: i32,
pub i_dot: i16,
pub cuc: i16,
pub cus: i16,
pub crc: i16,
pub crs: i16,
pub cic: i16,
pub cis: i16,
pub toe: u16,
pub af0: i32,
pub af1: i32,
pub af2: i8,
pub sisaindex_e1_e5b: u8,
pub toc: u16,
pub bgd_e1_e5b: i16,
pub reserved1: Vec<u8>,
pub health_e1b: u8,
pub data_validity_e1b: u8,
pub health_e5b: u8,
pub data_validity_e5b: u8,
pub reserved2: Vec<u8>,

                          }

                          impl WrappedTypesupport for MgaGAL { 

            type CStruct = ublox_msgs__msg__MgaGAL; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__MgaGAL() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__MgaGAL {

                unsafe { ublox_msgs__msg__MgaGAL__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__MgaGAL) -> () {

                unsafe { ublox_msgs__msg__MgaGAL__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> MgaGAL {
  MgaGAL {
type_: msg.type_,
version: msg.version,
svid: msg.svid,
reserved0: msg.reserved0,
iod_nav: msg.iod_nav,
delta_n: msg.delta_n,
m0: msg.m0,
e: msg.e,
sqrt_a: msg.sqrt_a,
omega0: msg.omega0,
i0: msg.i0,
omega: msg.omega,
omega_dot: msg.omega_dot,
i_dot: msg.i_dot,
cuc: msg.cuc,
cus: msg.cus,
crc: msg.crc,
crs: msg.crs,
cic: msg.cic,
cis: msg.cis,
toe: msg.toe,
af0: msg.af0,
af1: msg.af1,
af2: msg.af2,
sisaindex_e1_e5b: msg.sisaindex_e1_e5b,
toc: msg.toc,
bgd_e1_e5b: msg.bgd_e1_e5b,
// is_upper_bound_: false
// member.array_size_ : 2
reserved1: msg.reserved1.to_vec(),
health_e1b: msg.health_e1b,
data_validity_e1b: msg.data_validity_e1b,
health_e5b: msg.health_e5b,
data_validity_e5b: msg.data_validity_e5b,
// is_upper_bound_: false
// member.array_size_ : 4
reserved2: msg.reserved2.to_vec(),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.type_ = self.type_;
msg.version = self.version;
msg.svid = self.svid;
msg.reserved0 = self.reserved0;
msg.iod_nav = self.iod_nav;
msg.delta_n = self.delta_n;
msg.m0 = self.m0;
msg.e = self.e;
msg.sqrt_a = self.sqrt_a;
msg.omega0 = self.omega0;
msg.i0 = self.i0;
msg.omega = self.omega;
msg.omega_dot = self.omega_dot;
msg.i_dot = self.i_dot;
msg.cuc = self.cuc;
msg.cus = self.cus;
msg.crc = self.crc;
msg.crs = self.crs;
msg.cic = self.cic;
msg.cis = self.cis;
msg.toe = self.toe;
msg.af0 = self.af0;
msg.af1 = self.af1;
msg.af2 = self.af2;
msg.sisaindex_e1_e5b = self.sisaindex_e1_e5b;
msg.toc = self.toc;
msg.bgd_e1_e5b = self.bgd_e1_e5b;
assert_eq!(self.reserved1.len(), 2, "Field {} is fixed size of {}!", "reserved1", 2);
msg.reserved1.copy_from_slice(&self.reserved1[..2]);
msg.health_e1b = self.health_e1b;
msg.data_validity_e1b = self.data_validity_e1b;
msg.health_e5b = self.health_e5b;
msg.data_validity_e5b = self.data_validity_e5b;
assert_eq!(self.reserved2.len(), 4, "Field {} is fixed size of {}!", "reserved2", 4);
msg.reserved2.copy_from_slice(&self.reserved2[..4]);
}



        }


                          
                          impl Default for MgaGAL {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<MgaGAL>::new();
                                  MgaGAL::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct MonGNSS {

                              pub version: u8,
pub supported: u8,
pub default_gnss: u8,
pub enabled: u8,
pub simultaneous: u8,
pub reserved1: Vec<u8>,

                          }

                          impl WrappedTypesupport for MonGNSS { 

            type CStruct = ublox_msgs__msg__MonGNSS; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__MonGNSS() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__MonGNSS {

                unsafe { ublox_msgs__msg__MonGNSS__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__MonGNSS) -> () {

                unsafe { ublox_msgs__msg__MonGNSS__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> MonGNSS {
  MonGNSS {
version: msg.version,
supported: msg.supported,
default_gnss: msg.default_gnss,
enabled: msg.enabled,
simultaneous: msg.simultaneous,
// is_upper_bound_: false
// member.array_size_ : 3
reserved1: msg.reserved1.to_vec(),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.version = self.version;
msg.supported = self.supported;
msg.default_gnss = self.default_gnss;
msg.enabled = self.enabled;
msg.simultaneous = self.simultaneous;
assert_eq!(self.reserved1.len(), 3, "Field {} is fixed size of {}!", "reserved1", 3);
msg.reserved1.copy_from_slice(&self.reserved1[..3]);
}



        }


                          
                          impl Default for MonGNSS {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<MonGNSS>::new();
                                  MonGNSS::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct MonHW {

                              pub pin_sel: u32,
pub pin_bank: u32,
pub pin_dir: u32,
pub pin_val: u32,
pub noise_per_ms: u16,
pub agc_cnt: u16,
pub a_status: u8,
pub a_power: u8,
pub flags: u8,
pub reserved0: u8,
pub used_mask: u32,
pub vp: Vec<u8>,
pub jam_ind: u8,
pub reserved1: Vec<u8>,
pub pin_irq: u32,
pub pull_h: u32,
pub pull_l: u32,

                          }

                          impl WrappedTypesupport for MonHW { 

            type CStruct = ublox_msgs__msg__MonHW; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__MonHW() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__MonHW {

                unsafe { ublox_msgs__msg__MonHW__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__MonHW) -> () {

                unsafe { ublox_msgs__msg__MonHW__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> MonHW {
  MonHW {
pin_sel: msg.pin_sel,
pin_bank: msg.pin_bank,
pin_dir: msg.pin_dir,
pin_val: msg.pin_val,
noise_per_ms: msg.noise_per_ms,
agc_cnt: msg.agc_cnt,
a_status: msg.a_status,
a_power: msg.a_power,
flags: msg.flags,
reserved0: msg.reserved0,
used_mask: msg.used_mask,
// is_upper_bound_: false
// member.array_size_ : 17
vp: msg.vp.to_vec(),
jam_ind: msg.jam_ind,
// is_upper_bound_: false
// member.array_size_ : 2
reserved1: msg.reserved1.to_vec(),
pin_irq: msg.pin_irq,
pull_h: msg.pull_h,
pull_l: msg.pull_l,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.pin_sel = self.pin_sel;
msg.pin_bank = self.pin_bank;
msg.pin_dir = self.pin_dir;
msg.pin_val = self.pin_val;
msg.noise_per_ms = self.noise_per_ms;
msg.agc_cnt = self.agc_cnt;
msg.a_status = self.a_status;
msg.a_power = self.a_power;
msg.flags = self.flags;
msg.reserved0 = self.reserved0;
msg.used_mask = self.used_mask;
assert_eq!(self.vp.len(), 17, "Field {} is fixed size of {}!", "vp", 17);
msg.vp.copy_from_slice(&self.vp[..17]);
msg.jam_ind = self.jam_ind;
assert_eq!(self.reserved1.len(), 2, "Field {} is fixed size of {}!", "reserved1", 2);
msg.reserved1.copy_from_slice(&self.reserved1[..2]);
msg.pin_irq = self.pin_irq;
msg.pull_h = self.pull_h;
msg.pull_l = self.pull_l;
}



        }


                          
                          impl Default for MonHW {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<MonHW>::new();
                                  MonHW::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct MonHW6 {

                              pub pin_sel: u32,
pub pin_bank: u32,
pub pin_dir: u32,
pub pin_val: u32,
pub noise_per_ms: u16,
pub agc_cnt: u16,
pub a_status: u8,
pub a_power: u8,
pub flags: u8,
pub reserved0: u8,
pub used_mask: u32,
pub vp: Vec<u8>,
pub jam_ind: u8,
pub reserved1: Vec<u8>,
pub pin_irq: u32,
pub pull_h: u32,
pub pull_l: u32,

                          }

                          impl WrappedTypesupport for MonHW6 { 

            type CStruct = ublox_msgs__msg__MonHW6; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__MonHW6() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__MonHW6 {

                unsafe { ublox_msgs__msg__MonHW6__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__MonHW6) -> () {

                unsafe { ublox_msgs__msg__MonHW6__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> MonHW6 {
  MonHW6 {
pin_sel: msg.pin_sel,
pin_bank: msg.pin_bank,
pin_dir: msg.pin_dir,
pin_val: msg.pin_val,
noise_per_ms: msg.noise_per_ms,
agc_cnt: msg.agc_cnt,
a_status: msg.a_status,
a_power: msg.a_power,
flags: msg.flags,
reserved0: msg.reserved0,
used_mask: msg.used_mask,
// is_upper_bound_: false
// member.array_size_ : 25
vp: msg.vp.to_vec(),
jam_ind: msg.jam_ind,
// is_upper_bound_: false
// member.array_size_ : 2
reserved1: msg.reserved1.to_vec(),
pin_irq: msg.pin_irq,
pull_h: msg.pull_h,
pull_l: msg.pull_l,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.pin_sel = self.pin_sel;
msg.pin_bank = self.pin_bank;
msg.pin_dir = self.pin_dir;
msg.pin_val = self.pin_val;
msg.noise_per_ms = self.noise_per_ms;
msg.agc_cnt = self.agc_cnt;
msg.a_status = self.a_status;
msg.a_power = self.a_power;
msg.flags = self.flags;
msg.reserved0 = self.reserved0;
msg.used_mask = self.used_mask;
assert_eq!(self.vp.len(), 25, "Field {} is fixed size of {}!", "vp", 25);
msg.vp.copy_from_slice(&self.vp[..25]);
msg.jam_ind = self.jam_ind;
assert_eq!(self.reserved1.len(), 2, "Field {} is fixed size of {}!", "reserved1", 2);
msg.reserved1.copy_from_slice(&self.reserved1[..2]);
msg.pin_irq = self.pin_irq;
msg.pull_h = self.pull_h;
msg.pull_l = self.pull_l;
}



        }


                          
                          impl Default for MonHW6 {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<MonHW6>::new();
                                  MonHW6::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct MonVER {

                              pub sw_version: Vec<u8>,
pub hw_version: Vec<u8>,
pub extension: Vec<ublox_msgs::msg::MonVERExtension>,

                          }

                          impl WrappedTypesupport for MonVER { 

            type CStruct = ublox_msgs__msg__MonVER; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__MonVER() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__MonVER {

                unsafe { ublox_msgs__msg__MonVER__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__MonVER) -> () {

                unsafe { ublox_msgs__msg__MonVER__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> MonVER {
  MonVER {
// is_upper_bound_: false
// member.array_size_ : 30
sw_version: msg.sw_version.to_vec(),
// is_upper_bound_: false
// member.array_size_ : 10
hw_version: msg.hw_version.to_vec(),
// is_upper_bound_: false
// member.array_size_ : 0
extension : {
let mut temp = Vec::with_capacity(msg.extension.size);
let slice = unsafe { std::slice::from_raw_parts(msg.extension.data, msg.extension.size)};
for s in slice { temp.push(ublox_msgs::msg::MonVERExtension::from_native(s)); }
temp },
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {assert_eq!(self.sw_version.len(), 30, "Field {} is fixed size of {}!", "sw_version", 30);
msg.sw_version.copy_from_slice(&self.sw_version[..30]);
assert_eq!(self.hw_version.len(), 10, "Field {} is fixed size of {}!", "hw_version", 10);
msg.hw_version.copy_from_slice(&self.hw_version[..10]);
unsafe { ublox_msgs__msg__MonVERExtension__Sequence__fini(&mut msg.extension) };
unsafe { ublox_msgs__msg__MonVERExtension__Sequence__init(&mut msg.extension, self.extension.len()) };
let slice = unsafe { std::slice::from_raw_parts_mut(msg.extension.data, msg.extension.size)};
for (t,s) in slice.iter_mut().zip(&self.extension) { s.copy_to_native(t);}
}



        }


                          
                          impl Default for MonVER {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<MonVER>::new();
                                  MonVER::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct MonVERExtension {

                              pub field: Vec<u8>,

                          }

                          impl WrappedTypesupport for MonVERExtension { 

            type CStruct = ublox_msgs__msg__MonVERExtension; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__MonVERExtension() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__MonVERExtension {

                unsafe { ublox_msgs__msg__MonVERExtension__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__MonVERExtension) -> () {

                unsafe { ublox_msgs__msg__MonVERExtension__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> MonVERExtension {
  MonVERExtension {
// is_upper_bound_: false
// member.array_size_ : 30
field: msg.field.to_vec(),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {assert_eq!(self.field.len(), 30, "Field {} is fixed size of {}!", "field", 30);
msg.field.copy_from_slice(&self.field[..30]);
}



        }


                          
                          impl Default for MonVERExtension {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<MonVERExtension>::new();
                                  MonVERExtension::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct NavATT {

                              pub i_tow: u32,
pub version: u8,
pub reserved0: Vec<u8>,
pub roll: i32,
pub pitch: i32,
pub heading: i32,
pub acc_roll: u32,
pub acc_pitch: u32,
pub acc_heading: u32,

                          }

                          impl WrappedTypesupport for NavATT { 

            type CStruct = ublox_msgs__msg__NavATT; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavATT() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__NavATT {

                unsafe { ublox_msgs__msg__NavATT__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__NavATT) -> () {

                unsafe { ublox_msgs__msg__NavATT__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> NavATT {
  NavATT {
i_tow: msg.i_tow,
version: msg.version,
// is_upper_bound_: false
// member.array_size_ : 3
reserved0: msg.reserved0.to_vec(),
roll: msg.roll,
pitch: msg.pitch,
heading: msg.heading,
acc_roll: msg.acc_roll,
acc_pitch: msg.acc_pitch,
acc_heading: msg.acc_heading,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.i_tow = self.i_tow;
msg.version = self.version;
assert_eq!(self.reserved0.len(), 3, "Field {} is fixed size of {}!", "reserved0", 3);
msg.reserved0.copy_from_slice(&self.reserved0[..3]);
msg.roll = self.roll;
msg.pitch = self.pitch;
msg.heading = self.heading;
msg.acc_roll = self.acc_roll;
msg.acc_pitch = self.acc_pitch;
msg.acc_heading = self.acc_heading;
}



        }


                          
                          impl Default for NavATT {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<NavATT>::new();
                                  NavATT::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct NavCLOCK {

                              pub i_tow: u32,
pub clk_b: i32,
pub clk_d: i32,
pub t_acc: u32,
pub f_acc: u32,

                          }

                          impl WrappedTypesupport for NavCLOCK { 

            type CStruct = ublox_msgs__msg__NavCLOCK; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavCLOCK() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__NavCLOCK {

                unsafe { ublox_msgs__msg__NavCLOCK__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__NavCLOCK) -> () {

                unsafe { ublox_msgs__msg__NavCLOCK__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> NavCLOCK {
  NavCLOCK {
i_tow: msg.i_tow,
clk_b: msg.clk_b,
clk_d: msg.clk_d,
t_acc: msg.t_acc,
f_acc: msg.f_acc,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.i_tow = self.i_tow;
msg.clk_b = self.clk_b;
msg.clk_d = self.clk_d;
msg.t_acc = self.t_acc;
msg.f_acc = self.f_acc;
}



        }


                          
                          impl Default for NavCLOCK {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<NavCLOCK>::new();
                                  NavCLOCK::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct NavDGPS {

                              pub i_tow: u32,
pub age: i32,
pub base_id: i16,
pub base_health: i16,
pub num_ch: i8,
pub status: u8,
pub reserved1: u16,
pub sv: Vec<ublox_msgs::msg::NavDGPSSV>,

                          }

                          impl WrappedTypesupport for NavDGPS { 

            type CStruct = ublox_msgs__msg__NavDGPS; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavDGPS() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__NavDGPS {

                unsafe { ublox_msgs__msg__NavDGPS__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__NavDGPS) -> () {

                unsafe { ublox_msgs__msg__NavDGPS__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> NavDGPS {
  NavDGPS {
i_tow: msg.i_tow,
age: msg.age,
base_id: msg.base_id,
base_health: msg.base_health,
num_ch: msg.num_ch,
status: msg.status,
reserved1: msg.reserved1,
// is_upper_bound_: false
// member.array_size_ : 0
sv : {
let mut temp = Vec::with_capacity(msg.sv.size);
let slice = unsafe { std::slice::from_raw_parts(msg.sv.data, msg.sv.size)};
for s in slice { temp.push(ublox_msgs::msg::NavDGPSSV::from_native(s)); }
temp },
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.i_tow = self.i_tow;
msg.age = self.age;
msg.base_id = self.base_id;
msg.base_health = self.base_health;
msg.num_ch = self.num_ch;
msg.status = self.status;
msg.reserved1 = self.reserved1;
unsafe { ublox_msgs__msg__NavDGPSSV__Sequence__fini(&mut msg.sv) };
unsafe { ublox_msgs__msg__NavDGPSSV__Sequence__init(&mut msg.sv, self.sv.len()) };
let slice = unsafe { std::slice::from_raw_parts_mut(msg.sv.data, msg.sv.size)};
for (t,s) in slice.iter_mut().zip(&self.sv) { s.copy_to_native(t);}
}



        }


                          
                          impl Default for NavDGPS {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<NavDGPS>::new();
                                  NavDGPS::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct NavDGPSSV {

                              pub svid: u8,
pub flags: u8,
pub age_c: u16,
pub prc: f32,
pub prrc: f32,

                          }

                          impl WrappedTypesupport for NavDGPSSV { 

            type CStruct = ublox_msgs__msg__NavDGPSSV; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavDGPSSV() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__NavDGPSSV {

                unsafe { ublox_msgs__msg__NavDGPSSV__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__NavDGPSSV) -> () {

                unsafe { ublox_msgs__msg__NavDGPSSV__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> NavDGPSSV {
  NavDGPSSV {
svid: msg.svid,
flags: msg.flags,
age_c: msg.age_c,
prc: msg.prc,
prrc: msg.prrc,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.svid = self.svid;
msg.flags = self.flags;
msg.age_c = self.age_c;
msg.prc = self.prc;
msg.prrc = self.prrc;
}



        }


                          
                          impl Default for NavDGPSSV {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<NavDGPSSV>::new();
                                  NavDGPSSV::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct NavDOP {

                              pub i_tow: u32,
pub g_dop: u16,
pub p_dop: u16,
pub t_dop: u16,
pub v_dop: u16,
pub h_dop: u16,
pub n_dop: u16,
pub e_dop: u16,

                          }

                          impl WrappedTypesupport for NavDOP { 

            type CStruct = ublox_msgs__msg__NavDOP; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavDOP() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__NavDOP {

                unsafe { ublox_msgs__msg__NavDOP__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__NavDOP) -> () {

                unsafe { ublox_msgs__msg__NavDOP__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> NavDOP {
  NavDOP {
i_tow: msg.i_tow,
g_dop: msg.g_dop,
p_dop: msg.p_dop,
t_dop: msg.t_dop,
v_dop: msg.v_dop,
h_dop: msg.h_dop,
n_dop: msg.n_dop,
e_dop: msg.e_dop,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.i_tow = self.i_tow;
msg.g_dop = self.g_dop;
msg.p_dop = self.p_dop;
msg.t_dop = self.t_dop;
msg.v_dop = self.v_dop;
msg.h_dop = self.h_dop;
msg.n_dop = self.n_dop;
msg.e_dop = self.e_dop;
}



        }


                          
                          impl Default for NavDOP {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<NavDOP>::new();
                                  NavDOP::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct NavPOSECEF {

                              pub i_tow: u32,
pub ecef_x: i32,
pub ecef_y: i32,
pub ecef_z: i32,
pub p_acc: u32,

                          }

                          impl WrappedTypesupport for NavPOSECEF { 

            type CStruct = ublox_msgs__msg__NavPOSECEF; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavPOSECEF() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__NavPOSECEF {

                unsafe { ublox_msgs__msg__NavPOSECEF__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__NavPOSECEF) -> () {

                unsafe { ublox_msgs__msg__NavPOSECEF__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> NavPOSECEF {
  NavPOSECEF {
i_tow: msg.i_tow,
ecef_x: msg.ecef_x,
ecef_y: msg.ecef_y,
ecef_z: msg.ecef_z,
p_acc: msg.p_acc,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.i_tow = self.i_tow;
msg.ecef_x = self.ecef_x;
msg.ecef_y = self.ecef_y;
msg.ecef_z = self.ecef_z;
msg.p_acc = self.p_acc;
}



        }


                          
                          impl Default for NavPOSECEF {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<NavPOSECEF>::new();
                                  NavPOSECEF::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct NavPOSLLH {

                              pub i_tow: u32,
pub lon: i32,
pub lat: i32,
pub height: i32,
pub h_msl: i32,
pub h_acc: u32,
pub v_acc: u32,

                          }

                          impl WrappedTypesupport for NavPOSLLH { 

            type CStruct = ublox_msgs__msg__NavPOSLLH; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavPOSLLH() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__NavPOSLLH {

                unsafe { ublox_msgs__msg__NavPOSLLH__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__NavPOSLLH) -> () {

                unsafe { ublox_msgs__msg__NavPOSLLH__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> NavPOSLLH {
  NavPOSLLH {
i_tow: msg.i_tow,
lon: msg.lon,
lat: msg.lat,
height: msg.height,
h_msl: msg.h_msl,
h_acc: msg.h_acc,
v_acc: msg.v_acc,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.i_tow = self.i_tow;
msg.lon = self.lon;
msg.lat = self.lat;
msg.height = self.height;
msg.h_msl = self.h_msl;
msg.h_acc = self.h_acc;
msg.v_acc = self.v_acc;
}



        }


                          
                          impl Default for NavPOSLLH {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<NavPOSLLH>::new();
                                  NavPOSLLH::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct NavPVT {

                              pub i_tow: u32,
pub year: u16,
pub month: u8,
pub day: u8,
pub hour: u8,
pub min: u8,
pub sec: u8,
pub valid: u8,
pub t_acc: u32,
pub nano: i32,
pub fix_type: u8,
pub flags: u8,
pub flags2: u8,
pub num_sv: u8,
pub lon: i32,
pub lat: i32,
pub height: i32,
pub h_msl: i32,
pub h_acc: u32,
pub v_acc: u32,
pub vel_n: i32,
pub vel_e: i32,
pub vel_d: i32,
pub g_speed: i32,
pub heading: i32,
pub s_acc: u32,
pub head_acc: u32,
pub p_dop: u16,
pub reserved1: Vec<u8>,
pub head_veh: i32,
pub mag_dec: i16,
pub mag_acc: u16,

                          }

                          impl WrappedTypesupport for NavPVT { 

            type CStruct = ublox_msgs__msg__NavPVT; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavPVT() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__NavPVT {

                unsafe { ublox_msgs__msg__NavPVT__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__NavPVT) -> () {

                unsafe { ublox_msgs__msg__NavPVT__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> NavPVT {
  NavPVT {
i_tow: msg.i_tow,
year: msg.year,
month: msg.month,
day: msg.day,
hour: msg.hour,
min: msg.min,
sec: msg.sec,
valid: msg.valid,
t_acc: msg.t_acc,
nano: msg.nano,
fix_type: msg.fix_type,
flags: msg.flags,
flags2: msg.flags2,
num_sv: msg.num_sv,
lon: msg.lon,
lat: msg.lat,
height: msg.height,
h_msl: msg.h_msl,
h_acc: msg.h_acc,
v_acc: msg.v_acc,
vel_n: msg.vel_n,
vel_e: msg.vel_e,
vel_d: msg.vel_d,
g_speed: msg.g_speed,
heading: msg.heading,
s_acc: msg.s_acc,
head_acc: msg.head_acc,
p_dop: msg.p_dop,
// is_upper_bound_: false
// member.array_size_ : 6
reserved1: msg.reserved1.to_vec(),
head_veh: msg.head_veh,
mag_dec: msg.mag_dec,
mag_acc: msg.mag_acc,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.i_tow = self.i_tow;
msg.year = self.year;
msg.month = self.month;
msg.day = self.day;
msg.hour = self.hour;
msg.min = self.min;
msg.sec = self.sec;
msg.valid = self.valid;
msg.t_acc = self.t_acc;
msg.nano = self.nano;
msg.fix_type = self.fix_type;
msg.flags = self.flags;
msg.flags2 = self.flags2;
msg.num_sv = self.num_sv;
msg.lon = self.lon;
msg.lat = self.lat;
msg.height = self.height;
msg.h_msl = self.h_msl;
msg.h_acc = self.h_acc;
msg.v_acc = self.v_acc;
msg.vel_n = self.vel_n;
msg.vel_e = self.vel_e;
msg.vel_d = self.vel_d;
msg.g_speed = self.g_speed;
msg.heading = self.heading;
msg.s_acc = self.s_acc;
msg.head_acc = self.head_acc;
msg.p_dop = self.p_dop;
assert_eq!(self.reserved1.len(), 6, "Field {} is fixed size of {}!", "reserved1", 6);
msg.reserved1.copy_from_slice(&self.reserved1[..6]);
msg.head_veh = self.head_veh;
msg.mag_dec = self.mag_dec;
msg.mag_acc = self.mag_acc;
}



        }


                          
                          impl Default for NavPVT {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<NavPVT>::new();
                                  NavPVT::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct NavPVT7 {

                              pub i_tow: u32,
pub year: u16,
pub month: u8,
pub day: u8,
pub hour: u8,
pub min: u8,
pub sec: u8,
pub valid: u8,
pub t_acc: u32,
pub nano: i32,
pub fix_type: u8,
pub flags: u8,
pub flags2: u8,
pub num_sv: u8,
pub lon: i32,
pub lat: i32,
pub height: i32,
pub h_msl: i32,
pub h_acc: u32,
pub v_acc: u32,
pub vel_n: i32,
pub vel_e: i32,
pub vel_d: i32,
pub g_speed: i32,
pub heading: i32,
pub s_acc: u32,
pub head_acc: u32,
pub p_dop: u16,
pub reserved1: Vec<u8>,

                          }

                          impl WrappedTypesupport for NavPVT7 { 

            type CStruct = ublox_msgs__msg__NavPVT7; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavPVT7() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__NavPVT7 {

                unsafe { ublox_msgs__msg__NavPVT7__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__NavPVT7) -> () {

                unsafe { ublox_msgs__msg__NavPVT7__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> NavPVT7 {
  NavPVT7 {
i_tow: msg.i_tow,
year: msg.year,
month: msg.month,
day: msg.day,
hour: msg.hour,
min: msg.min,
sec: msg.sec,
valid: msg.valid,
t_acc: msg.t_acc,
nano: msg.nano,
fix_type: msg.fix_type,
flags: msg.flags,
flags2: msg.flags2,
num_sv: msg.num_sv,
lon: msg.lon,
lat: msg.lat,
height: msg.height,
h_msl: msg.h_msl,
h_acc: msg.h_acc,
v_acc: msg.v_acc,
vel_n: msg.vel_n,
vel_e: msg.vel_e,
vel_d: msg.vel_d,
g_speed: msg.g_speed,
heading: msg.heading,
s_acc: msg.s_acc,
head_acc: msg.head_acc,
p_dop: msg.p_dop,
// is_upper_bound_: false
// member.array_size_ : 6
reserved1: msg.reserved1.to_vec(),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.i_tow = self.i_tow;
msg.year = self.year;
msg.month = self.month;
msg.day = self.day;
msg.hour = self.hour;
msg.min = self.min;
msg.sec = self.sec;
msg.valid = self.valid;
msg.t_acc = self.t_acc;
msg.nano = self.nano;
msg.fix_type = self.fix_type;
msg.flags = self.flags;
msg.flags2 = self.flags2;
msg.num_sv = self.num_sv;
msg.lon = self.lon;
msg.lat = self.lat;
msg.height = self.height;
msg.h_msl = self.h_msl;
msg.h_acc = self.h_acc;
msg.v_acc = self.v_acc;
msg.vel_n = self.vel_n;
msg.vel_e = self.vel_e;
msg.vel_d = self.vel_d;
msg.g_speed = self.g_speed;
msg.heading = self.heading;
msg.s_acc = self.s_acc;
msg.head_acc = self.head_acc;
msg.p_dop = self.p_dop;
assert_eq!(self.reserved1.len(), 6, "Field {} is fixed size of {}!", "reserved1", 6);
msg.reserved1.copy_from_slice(&self.reserved1[..6]);
}



        }


                          
                          impl Default for NavPVT7 {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<NavPVT7>::new();
                                  NavPVT7::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct NavRELPOSNED {

                              pub version: u8,
pub reserved0: u8,
pub ref_station_id: u16,
pub i_tow: u32,
pub rel_pos_n: i32,
pub rel_pos_e: i32,
pub rel_pos_d: i32,
pub rel_pos_hpn: i8,
pub rel_pos_hpe: i8,
pub rel_pos_hpd: i8,
pub reserved1: u8,
pub acc_n: u32,
pub acc_e: u32,
pub acc_d: u32,
pub flags: u32,

                          }

                          impl WrappedTypesupport for NavRELPOSNED { 

            type CStruct = ublox_msgs__msg__NavRELPOSNED; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavRELPOSNED() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__NavRELPOSNED {

                unsafe { ublox_msgs__msg__NavRELPOSNED__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__NavRELPOSNED) -> () {

                unsafe { ublox_msgs__msg__NavRELPOSNED__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> NavRELPOSNED {
  NavRELPOSNED {
version: msg.version,
reserved0: msg.reserved0,
ref_station_id: msg.ref_station_id,
i_tow: msg.i_tow,
rel_pos_n: msg.rel_pos_n,
rel_pos_e: msg.rel_pos_e,
rel_pos_d: msg.rel_pos_d,
rel_pos_hpn: msg.rel_pos_hpn,
rel_pos_hpe: msg.rel_pos_hpe,
rel_pos_hpd: msg.rel_pos_hpd,
reserved1: msg.reserved1,
acc_n: msg.acc_n,
acc_e: msg.acc_e,
acc_d: msg.acc_d,
flags: msg.flags,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.version = self.version;
msg.reserved0 = self.reserved0;
msg.ref_station_id = self.ref_station_id;
msg.i_tow = self.i_tow;
msg.rel_pos_n = self.rel_pos_n;
msg.rel_pos_e = self.rel_pos_e;
msg.rel_pos_d = self.rel_pos_d;
msg.rel_pos_hpn = self.rel_pos_hpn;
msg.rel_pos_hpe = self.rel_pos_hpe;
msg.rel_pos_hpd = self.rel_pos_hpd;
msg.reserved1 = self.reserved1;
msg.acc_n = self.acc_n;
msg.acc_e = self.acc_e;
msg.acc_d = self.acc_d;
msg.flags = self.flags;
}



        }


                          
                          impl Default for NavRELPOSNED {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<NavRELPOSNED>::new();
                                  NavRELPOSNED::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct NavRELPOSNED9 {

                              pub version: u8,
pub reserved1: u8,
pub ref_station_id: u16,
pub i_tow: u32,
pub rel_pos_n: i32,
pub rel_pos_e: i32,
pub rel_pos_d: i32,
pub rel_pos_length: i32,
pub rel_pos_heading: i32,
pub reserved2: Vec<u8>,
pub rel_pos_hpn: i8,
pub rel_pos_hpe: i8,
pub rel_pos_hpd: i8,
pub rel_pos_hp_length: i8,
pub acc_n: u32,
pub acc_e: u32,
pub acc_d: u32,
pub acc_length: u32,
pub acc_heading: u32,
pub reserved3: Vec<u8>,
pub flags: u32,

                          }

                          impl WrappedTypesupport for NavRELPOSNED9 { 

            type CStruct = ublox_msgs__msg__NavRELPOSNED9; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavRELPOSNED9() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__NavRELPOSNED9 {

                unsafe { ublox_msgs__msg__NavRELPOSNED9__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__NavRELPOSNED9) -> () {

                unsafe { ublox_msgs__msg__NavRELPOSNED9__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> NavRELPOSNED9 {
  NavRELPOSNED9 {
version: msg.version,
reserved1: msg.reserved1,
ref_station_id: msg.ref_station_id,
i_tow: msg.i_tow,
rel_pos_n: msg.rel_pos_n,
rel_pos_e: msg.rel_pos_e,
rel_pos_d: msg.rel_pos_d,
rel_pos_length: msg.rel_pos_length,
rel_pos_heading: msg.rel_pos_heading,
// is_upper_bound_: false
// member.array_size_ : 4
reserved2: msg.reserved2.to_vec(),
rel_pos_hpn: msg.rel_pos_hpn,
rel_pos_hpe: msg.rel_pos_hpe,
rel_pos_hpd: msg.rel_pos_hpd,
rel_pos_hp_length: msg.rel_pos_hp_length,
acc_n: msg.acc_n,
acc_e: msg.acc_e,
acc_d: msg.acc_d,
acc_length: msg.acc_length,
acc_heading: msg.acc_heading,
// is_upper_bound_: false
// member.array_size_ : 4
reserved3: msg.reserved3.to_vec(),
flags: msg.flags,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.version = self.version;
msg.reserved1 = self.reserved1;
msg.ref_station_id = self.ref_station_id;
msg.i_tow = self.i_tow;
msg.rel_pos_n = self.rel_pos_n;
msg.rel_pos_e = self.rel_pos_e;
msg.rel_pos_d = self.rel_pos_d;
msg.rel_pos_length = self.rel_pos_length;
msg.rel_pos_heading = self.rel_pos_heading;
assert_eq!(self.reserved2.len(), 4, "Field {} is fixed size of {}!", "reserved2", 4);
msg.reserved2.copy_from_slice(&self.reserved2[..4]);
msg.rel_pos_hpn = self.rel_pos_hpn;
msg.rel_pos_hpe = self.rel_pos_hpe;
msg.rel_pos_hpd = self.rel_pos_hpd;
msg.rel_pos_hp_length = self.rel_pos_hp_length;
msg.acc_n = self.acc_n;
msg.acc_e = self.acc_e;
msg.acc_d = self.acc_d;
msg.acc_length = self.acc_length;
msg.acc_heading = self.acc_heading;
assert_eq!(self.reserved3.len(), 4, "Field {} is fixed size of {}!", "reserved3", 4);
msg.reserved3.copy_from_slice(&self.reserved3[..4]);
msg.flags = self.flags;
}



        }


                          
                          impl Default for NavRELPOSNED9 {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<NavRELPOSNED9>::new();
                                  NavRELPOSNED9::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct NavSAT {

                              pub i_tow: u32,
pub version: u8,
pub num_svs: u8,
pub reserved0: Vec<u8>,
pub sv: Vec<ublox_msgs::msg::NavSATSV>,

                          }

                          impl WrappedTypesupport for NavSAT { 

            type CStruct = ublox_msgs__msg__NavSAT; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavSAT() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__NavSAT {

                unsafe { ublox_msgs__msg__NavSAT__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__NavSAT) -> () {

                unsafe { ublox_msgs__msg__NavSAT__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> NavSAT {
  NavSAT {
i_tow: msg.i_tow,
version: msg.version,
num_svs: msg.num_svs,
// is_upper_bound_: false
// member.array_size_ : 2
reserved0: msg.reserved0.to_vec(),
// is_upper_bound_: false
// member.array_size_ : 0
sv : {
let mut temp = Vec::with_capacity(msg.sv.size);
let slice = unsafe { std::slice::from_raw_parts(msg.sv.data, msg.sv.size)};
for s in slice { temp.push(ublox_msgs::msg::NavSATSV::from_native(s)); }
temp },
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.i_tow = self.i_tow;
msg.version = self.version;
msg.num_svs = self.num_svs;
assert_eq!(self.reserved0.len(), 2, "Field {} is fixed size of {}!", "reserved0", 2);
msg.reserved0.copy_from_slice(&self.reserved0[..2]);
unsafe { ublox_msgs__msg__NavSATSV__Sequence__fini(&mut msg.sv) };
unsafe { ublox_msgs__msg__NavSATSV__Sequence__init(&mut msg.sv, self.sv.len()) };
let slice = unsafe { std::slice::from_raw_parts_mut(msg.sv.data, msg.sv.size)};
for (t,s) in slice.iter_mut().zip(&self.sv) { s.copy_to_native(t);}
}



        }


                          
                          impl Default for NavSAT {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<NavSAT>::new();
                                  NavSAT::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct NavSATSV {

                              pub gnss_id: u8,
pub sv_id: u8,
pub cno: u8,
pub elev: i8,
pub azim: i16,
pub pr_res: i16,
pub flags: u32,

                          }

                          impl WrappedTypesupport for NavSATSV { 

            type CStruct = ublox_msgs__msg__NavSATSV; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavSATSV() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__NavSATSV {

                unsafe { ublox_msgs__msg__NavSATSV__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__NavSATSV) -> () {

                unsafe { ublox_msgs__msg__NavSATSV__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> NavSATSV {
  NavSATSV {
gnss_id: msg.gnss_id,
sv_id: msg.sv_id,
cno: msg.cno,
elev: msg.elev,
azim: msg.azim,
pr_res: msg.pr_res,
flags: msg.flags,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.gnss_id = self.gnss_id;
msg.sv_id = self.sv_id;
msg.cno = self.cno;
msg.elev = self.elev;
msg.azim = self.azim;
msg.pr_res = self.pr_res;
msg.flags = self.flags;
}



        }


                          
                          impl Default for NavSATSV {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<NavSATSV>::new();
                                  NavSATSV::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct NavSBAS {

                              pub i_tow: u32,
pub geo: u8,
pub mode: u8,
pub sys: i8,
pub service: u8,
pub cnt: u8,
pub reserved0: Vec<u8>,
pub sv: Vec<ublox_msgs::msg::NavSBASSV>,

                          }

                          impl WrappedTypesupport for NavSBAS { 

            type CStruct = ublox_msgs__msg__NavSBAS; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavSBAS() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__NavSBAS {

                unsafe { ublox_msgs__msg__NavSBAS__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__NavSBAS) -> () {

                unsafe { ublox_msgs__msg__NavSBAS__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> NavSBAS {
  NavSBAS {
i_tow: msg.i_tow,
geo: msg.geo,
mode: msg.mode,
sys: msg.sys,
service: msg.service,
cnt: msg.cnt,
// is_upper_bound_: false
// member.array_size_ : 3
reserved0: msg.reserved0.to_vec(),
// is_upper_bound_: false
// member.array_size_ : 0
sv : {
let mut temp = Vec::with_capacity(msg.sv.size);
let slice = unsafe { std::slice::from_raw_parts(msg.sv.data, msg.sv.size)};
for s in slice { temp.push(ublox_msgs::msg::NavSBASSV::from_native(s)); }
temp },
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.i_tow = self.i_tow;
msg.geo = self.geo;
msg.mode = self.mode;
msg.sys = self.sys;
msg.service = self.service;
msg.cnt = self.cnt;
assert_eq!(self.reserved0.len(), 3, "Field {} is fixed size of {}!", "reserved0", 3);
msg.reserved0.copy_from_slice(&self.reserved0[..3]);
unsafe { ublox_msgs__msg__NavSBASSV__Sequence__fini(&mut msg.sv) };
unsafe { ublox_msgs__msg__NavSBASSV__Sequence__init(&mut msg.sv, self.sv.len()) };
let slice = unsafe { std::slice::from_raw_parts_mut(msg.sv.data, msg.sv.size)};
for (t,s) in slice.iter_mut().zip(&self.sv) { s.copy_to_native(t);}
}



        }


                          
                          impl Default for NavSBAS {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<NavSBAS>::new();
                                  NavSBAS::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct NavSBASSV {

                              pub svid: u8,
pub flags: u8,
pub udre: u8,
pub sv_sys: u8,
pub sv_service: u8,
pub reserved1: u8,
pub prc: i16,
pub reserved2: u16,
pub ic: i16,

                          }

                          impl WrappedTypesupport for NavSBASSV { 

            type CStruct = ublox_msgs__msg__NavSBASSV; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavSBASSV() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__NavSBASSV {

                unsafe { ublox_msgs__msg__NavSBASSV__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__NavSBASSV) -> () {

                unsafe { ublox_msgs__msg__NavSBASSV__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> NavSBASSV {
  NavSBASSV {
svid: msg.svid,
flags: msg.flags,
udre: msg.udre,
sv_sys: msg.sv_sys,
sv_service: msg.sv_service,
reserved1: msg.reserved1,
prc: msg.prc,
reserved2: msg.reserved2,
ic: msg.ic,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.svid = self.svid;
msg.flags = self.flags;
msg.udre = self.udre;
msg.sv_sys = self.sv_sys;
msg.sv_service = self.sv_service;
msg.reserved1 = self.reserved1;
msg.prc = self.prc;
msg.reserved2 = self.reserved2;
msg.ic = self.ic;
}



        }


                          
                          impl Default for NavSBASSV {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<NavSBASSV>::new();
                                  NavSBASSV::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct NavSOL {

                              pub i_tow: u32,
pub f_tow: i32,
pub week: i16,
pub gps_fix: u8,
pub flags: u8,
pub ecef_x: i32,
pub ecef_y: i32,
pub ecef_z: i32,
pub p_acc: u32,
pub ecef_vx: i32,
pub ecef_vy: i32,
pub ecef_vz: i32,
pub s_acc: u32,
pub p_dop: u16,
pub reserved1: u8,
pub num_sv: u8,
pub reserved2: u32,

                          }

                          impl WrappedTypesupport for NavSOL { 

            type CStruct = ublox_msgs__msg__NavSOL; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavSOL() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__NavSOL {

                unsafe { ublox_msgs__msg__NavSOL__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__NavSOL) -> () {

                unsafe { ublox_msgs__msg__NavSOL__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> NavSOL {
  NavSOL {
i_tow: msg.i_tow,
f_tow: msg.f_tow,
week: msg.week,
gps_fix: msg.gps_fix,
flags: msg.flags,
ecef_x: msg.ecef_x,
ecef_y: msg.ecef_y,
ecef_z: msg.ecef_z,
p_acc: msg.p_acc,
ecef_vx: msg.ecef_vx,
ecef_vy: msg.ecef_vy,
ecef_vz: msg.ecef_vz,
s_acc: msg.s_acc,
p_dop: msg.p_dop,
reserved1: msg.reserved1,
num_sv: msg.num_sv,
reserved2: msg.reserved2,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.i_tow = self.i_tow;
msg.f_tow = self.f_tow;
msg.week = self.week;
msg.gps_fix = self.gps_fix;
msg.flags = self.flags;
msg.ecef_x = self.ecef_x;
msg.ecef_y = self.ecef_y;
msg.ecef_z = self.ecef_z;
msg.p_acc = self.p_acc;
msg.ecef_vx = self.ecef_vx;
msg.ecef_vy = self.ecef_vy;
msg.ecef_vz = self.ecef_vz;
msg.s_acc = self.s_acc;
msg.p_dop = self.p_dop;
msg.reserved1 = self.reserved1;
msg.num_sv = self.num_sv;
msg.reserved2 = self.reserved2;
}



        }


                          
                          impl Default for NavSOL {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<NavSOL>::new();
                                  NavSOL::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct NavSTATUS {

                              pub i_tow: u32,
pub gps_fix: u8,
pub flags: u8,
pub fix_stat: u8,
pub flags2: u8,
pub ttff: u32,
pub msss: u32,

                          }

                          impl WrappedTypesupport for NavSTATUS { 

            type CStruct = ublox_msgs__msg__NavSTATUS; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavSTATUS() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__NavSTATUS {

                unsafe { ublox_msgs__msg__NavSTATUS__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__NavSTATUS) -> () {

                unsafe { ublox_msgs__msg__NavSTATUS__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> NavSTATUS {
  NavSTATUS {
i_tow: msg.i_tow,
gps_fix: msg.gps_fix,
flags: msg.flags,
fix_stat: msg.fix_stat,
flags2: msg.flags2,
ttff: msg.ttff,
msss: msg.msss,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.i_tow = self.i_tow;
msg.gps_fix = self.gps_fix;
msg.flags = self.flags;
msg.fix_stat = self.fix_stat;
msg.flags2 = self.flags2;
msg.ttff = self.ttff;
msg.msss = self.msss;
}



        }


                          
                          impl Default for NavSTATUS {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<NavSTATUS>::new();
                                  NavSTATUS::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct NavSVIN {

                              pub version: u8,
pub reserved0: Vec<u8>,
pub i_tow: u32,
pub dur: u32,
pub mean_x: i32,
pub mean_y: i32,
pub mean_z: i32,
pub mean_xhp: i8,
pub mean_yhp: i8,
pub mean_zhp: i8,
pub reserved1: u8,
pub mean_acc: u32,
pub obs: u32,
pub valid: u8,
pub active: u8,
pub reserved3: Vec<u8>,

                          }

                          impl WrappedTypesupport for NavSVIN { 

            type CStruct = ublox_msgs__msg__NavSVIN; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavSVIN() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__NavSVIN {

                unsafe { ublox_msgs__msg__NavSVIN__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__NavSVIN) -> () {

                unsafe { ublox_msgs__msg__NavSVIN__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> NavSVIN {
  NavSVIN {
version: msg.version,
// is_upper_bound_: false
// member.array_size_ : 3
reserved0: msg.reserved0.to_vec(),
i_tow: msg.i_tow,
dur: msg.dur,
mean_x: msg.mean_x,
mean_y: msg.mean_y,
mean_z: msg.mean_z,
mean_xhp: msg.mean_xhp,
mean_yhp: msg.mean_yhp,
mean_zhp: msg.mean_zhp,
reserved1: msg.reserved1,
mean_acc: msg.mean_acc,
obs: msg.obs,
valid: msg.valid,
active: msg.active,
// is_upper_bound_: false
// member.array_size_ : 2
reserved3: msg.reserved3.to_vec(),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.version = self.version;
assert_eq!(self.reserved0.len(), 3, "Field {} is fixed size of {}!", "reserved0", 3);
msg.reserved0.copy_from_slice(&self.reserved0[..3]);
msg.i_tow = self.i_tow;
msg.dur = self.dur;
msg.mean_x = self.mean_x;
msg.mean_y = self.mean_y;
msg.mean_z = self.mean_z;
msg.mean_xhp = self.mean_xhp;
msg.mean_yhp = self.mean_yhp;
msg.mean_zhp = self.mean_zhp;
msg.reserved1 = self.reserved1;
msg.mean_acc = self.mean_acc;
msg.obs = self.obs;
msg.valid = self.valid;
msg.active = self.active;
assert_eq!(self.reserved3.len(), 2, "Field {} is fixed size of {}!", "reserved3", 2);
msg.reserved3.copy_from_slice(&self.reserved3[..2]);
}



        }


                          
                          impl Default for NavSVIN {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<NavSVIN>::new();
                                  NavSVIN::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct NavSVINFO {

                              pub i_tow: u32,
pub num_ch: u8,
pub global_flags: u8,
pub reserved2: u16,
pub sv: Vec<ublox_msgs::msg::NavSVINFOSV>,

                          }

                          impl WrappedTypesupport for NavSVINFO { 

            type CStruct = ublox_msgs__msg__NavSVINFO; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavSVINFO() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__NavSVINFO {

                unsafe { ublox_msgs__msg__NavSVINFO__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__NavSVINFO) -> () {

                unsafe { ublox_msgs__msg__NavSVINFO__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> NavSVINFO {
  NavSVINFO {
i_tow: msg.i_tow,
num_ch: msg.num_ch,
global_flags: msg.global_flags,
reserved2: msg.reserved2,
// is_upper_bound_: false
// member.array_size_ : 0
sv : {
let mut temp = Vec::with_capacity(msg.sv.size);
let slice = unsafe { std::slice::from_raw_parts(msg.sv.data, msg.sv.size)};
for s in slice { temp.push(ublox_msgs::msg::NavSVINFOSV::from_native(s)); }
temp },
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.i_tow = self.i_tow;
msg.num_ch = self.num_ch;
msg.global_flags = self.global_flags;
msg.reserved2 = self.reserved2;
unsafe { ublox_msgs__msg__NavSVINFOSV__Sequence__fini(&mut msg.sv) };
unsafe { ublox_msgs__msg__NavSVINFOSV__Sequence__init(&mut msg.sv, self.sv.len()) };
let slice = unsafe { std::slice::from_raw_parts_mut(msg.sv.data, msg.sv.size)};
for (t,s) in slice.iter_mut().zip(&self.sv) { s.copy_to_native(t);}
}



        }


                          
                          impl Default for NavSVINFO {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<NavSVINFO>::new();
                                  NavSVINFO::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct NavSVINFOSV {

                              pub chn: u8,
pub svid: u8,
pub flags: u8,
pub quality: u8,
pub cno: u8,
pub elev: i8,
pub azim: i16,
pub pr_res: i32,

                          }

                          impl WrappedTypesupport for NavSVINFOSV { 

            type CStruct = ublox_msgs__msg__NavSVINFOSV; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavSVINFOSV() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__NavSVINFOSV {

                unsafe { ublox_msgs__msg__NavSVINFOSV__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__NavSVINFOSV) -> () {

                unsafe { ublox_msgs__msg__NavSVINFOSV__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> NavSVINFOSV {
  NavSVINFOSV {
chn: msg.chn,
svid: msg.svid,
flags: msg.flags,
quality: msg.quality,
cno: msg.cno,
elev: msg.elev,
azim: msg.azim,
pr_res: msg.pr_res,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.chn = self.chn;
msg.svid = self.svid;
msg.flags = self.flags;
msg.quality = self.quality;
msg.cno = self.cno;
msg.elev = self.elev;
msg.azim = self.azim;
msg.pr_res = self.pr_res;
}



        }


                          
                          impl Default for NavSVINFOSV {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<NavSVINFOSV>::new();
                                  NavSVINFOSV::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct NavTIMEGPS {

                              pub i_tow: u32,
pub f_tow: i32,
pub week: i16,
pub leap_s: i8,
pub valid: u8,
pub t_acc: u32,

                          }

                          impl WrappedTypesupport for NavTIMEGPS { 

            type CStruct = ublox_msgs__msg__NavTIMEGPS; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavTIMEGPS() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__NavTIMEGPS {

                unsafe { ublox_msgs__msg__NavTIMEGPS__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__NavTIMEGPS) -> () {

                unsafe { ublox_msgs__msg__NavTIMEGPS__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> NavTIMEGPS {
  NavTIMEGPS {
i_tow: msg.i_tow,
f_tow: msg.f_tow,
week: msg.week,
leap_s: msg.leap_s,
valid: msg.valid,
t_acc: msg.t_acc,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.i_tow = self.i_tow;
msg.f_tow = self.f_tow;
msg.week = self.week;
msg.leap_s = self.leap_s;
msg.valid = self.valid;
msg.t_acc = self.t_acc;
}



        }


                          
                          impl Default for NavTIMEGPS {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<NavTIMEGPS>::new();
                                  NavTIMEGPS::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct NavTIMEUTC {

                              pub i_tow: u32,
pub t_acc: u32,
pub nano: i32,
pub year: u16,
pub month: u8,
pub day: u8,
pub hour: u8,
pub min: u8,
pub sec: u8,
pub valid: u8,

                          }

                          impl WrappedTypesupport for NavTIMEUTC { 

            type CStruct = ublox_msgs__msg__NavTIMEUTC; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavTIMEUTC() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__NavTIMEUTC {

                unsafe { ublox_msgs__msg__NavTIMEUTC__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__NavTIMEUTC) -> () {

                unsafe { ublox_msgs__msg__NavTIMEUTC__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> NavTIMEUTC {
  NavTIMEUTC {
i_tow: msg.i_tow,
t_acc: msg.t_acc,
nano: msg.nano,
year: msg.year,
month: msg.month,
day: msg.day,
hour: msg.hour,
min: msg.min,
sec: msg.sec,
valid: msg.valid,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.i_tow = self.i_tow;
msg.t_acc = self.t_acc;
msg.nano = self.nano;
msg.year = self.year;
msg.month = self.month;
msg.day = self.day;
msg.hour = self.hour;
msg.min = self.min;
msg.sec = self.sec;
msg.valid = self.valid;
}



        }


                          
                          impl Default for NavTIMEUTC {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<NavTIMEUTC>::new();
                                  NavTIMEUTC::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct NavVELECEF {

                              pub i_tow: u32,
pub ecef_vx: i32,
pub ecef_vy: i32,
pub ecef_vz: i32,
pub s_acc: u32,

                          }

                          impl WrappedTypesupport for NavVELECEF { 

            type CStruct = ublox_msgs__msg__NavVELECEF; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavVELECEF() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__NavVELECEF {

                unsafe { ublox_msgs__msg__NavVELECEF__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__NavVELECEF) -> () {

                unsafe { ublox_msgs__msg__NavVELECEF__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> NavVELECEF {
  NavVELECEF {
i_tow: msg.i_tow,
ecef_vx: msg.ecef_vx,
ecef_vy: msg.ecef_vy,
ecef_vz: msg.ecef_vz,
s_acc: msg.s_acc,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.i_tow = self.i_tow;
msg.ecef_vx = self.ecef_vx;
msg.ecef_vy = self.ecef_vy;
msg.ecef_vz = self.ecef_vz;
msg.s_acc = self.s_acc;
}



        }


                          
                          impl Default for NavVELECEF {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<NavVELECEF>::new();
                                  NavVELECEF::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct NavVELNED {

                              pub i_tow: u32,
pub vel_n: i32,
pub vel_e: i32,
pub vel_d: i32,
pub speed: u32,
pub g_speed: u32,
pub heading: i32,
pub s_acc: u32,
pub c_acc: u32,

                          }

                          impl WrappedTypesupport for NavVELNED { 

            type CStruct = ublox_msgs__msg__NavVELNED; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavVELNED() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__NavVELNED {

                unsafe { ublox_msgs__msg__NavVELNED__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__NavVELNED) -> () {

                unsafe { ublox_msgs__msg__NavVELNED__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> NavVELNED {
  NavVELNED {
i_tow: msg.i_tow,
vel_n: msg.vel_n,
vel_e: msg.vel_e,
vel_d: msg.vel_d,
speed: msg.speed,
g_speed: msg.g_speed,
heading: msg.heading,
s_acc: msg.s_acc,
c_acc: msg.c_acc,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.i_tow = self.i_tow;
msg.vel_n = self.vel_n;
msg.vel_e = self.vel_e;
msg.vel_d = self.vel_d;
msg.speed = self.speed;
msg.g_speed = self.g_speed;
msg.heading = self.heading;
msg.s_acc = self.s_acc;
msg.c_acc = self.c_acc;
}



        }


                          
                          impl Default for NavVELNED {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<NavVELNED>::new();
                                  NavVELNED::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct RxmALM {

                              pub svid: u32,
pub week: u32,
pub dwrd: Vec<u32>,

                          }

                          impl WrappedTypesupport for RxmALM { 

            type CStruct = ublox_msgs__msg__RxmALM; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__RxmALM() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__RxmALM {

                unsafe { ublox_msgs__msg__RxmALM__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__RxmALM) -> () {

                unsafe { ublox_msgs__msg__RxmALM__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> RxmALM {
  RxmALM {
svid: msg.svid,
week: msg.week,
// is_upper_bound_: false
// member.array_size_ : 0
dwrd: msg.dwrd.to_vec(),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.svid = self.svid;
msg.week = self.week;
msg.dwrd.update(&self.dwrd);
}



        }


                          
                          impl Default for RxmALM {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<RxmALM>::new();
                                  RxmALM::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct RxmEPH {

                              pub svid: u32,
pub how: u32,
pub sf1d: Vec<u32>,
pub sf2d: Vec<u32>,
pub sf3d: Vec<u32>,

                          }

                          impl WrappedTypesupport for RxmEPH { 

            type CStruct = ublox_msgs__msg__RxmEPH; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__RxmEPH() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__RxmEPH {

                unsafe { ublox_msgs__msg__RxmEPH__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__RxmEPH) -> () {

                unsafe { ublox_msgs__msg__RxmEPH__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> RxmEPH {
  RxmEPH {
svid: msg.svid,
how: msg.how,
// is_upper_bound_: false
// member.array_size_ : 0
sf1d: msg.sf1d.to_vec(),
// is_upper_bound_: false
// member.array_size_ : 0
sf2d: msg.sf2d.to_vec(),
// is_upper_bound_: false
// member.array_size_ : 0
sf3d: msg.sf3d.to_vec(),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.svid = self.svid;
msg.how = self.how;
msg.sf1d.update(&self.sf1d);
msg.sf2d.update(&self.sf2d);
msg.sf3d.update(&self.sf3d);
}



        }


                          
                          impl Default for RxmEPH {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<RxmEPH>::new();
                                  RxmEPH::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct RxmRAW {

                              pub rcv_tow: i32,
pub week: i16,
pub num_sv: u8,
pub reserved1: u8,
pub sv: Vec<ublox_msgs::msg::RxmRAWSV>,

                          }

                          impl WrappedTypesupport for RxmRAW { 

            type CStruct = ublox_msgs__msg__RxmRAW; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__RxmRAW() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__RxmRAW {

                unsafe { ublox_msgs__msg__RxmRAW__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__RxmRAW) -> () {

                unsafe { ublox_msgs__msg__RxmRAW__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> RxmRAW {
  RxmRAW {
rcv_tow: msg.rcv_tow,
week: msg.week,
num_sv: msg.num_sv,
reserved1: msg.reserved1,
// is_upper_bound_: false
// member.array_size_ : 0
sv : {
let mut temp = Vec::with_capacity(msg.sv.size);
let slice = unsafe { std::slice::from_raw_parts(msg.sv.data, msg.sv.size)};
for s in slice { temp.push(ublox_msgs::msg::RxmRAWSV::from_native(s)); }
temp },
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.rcv_tow = self.rcv_tow;
msg.week = self.week;
msg.num_sv = self.num_sv;
msg.reserved1 = self.reserved1;
unsafe { ublox_msgs__msg__RxmRAWSV__Sequence__fini(&mut msg.sv) };
unsafe { ublox_msgs__msg__RxmRAWSV__Sequence__init(&mut msg.sv, self.sv.len()) };
let slice = unsafe { std::slice::from_raw_parts_mut(msg.sv.data, msg.sv.size)};
for (t,s) in slice.iter_mut().zip(&self.sv) { s.copy_to_native(t);}
}



        }


                          
                          impl Default for RxmRAW {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<RxmRAW>::new();
                                  RxmRAW::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct RxmRAWSV {

                              pub cp_mes: f64,
pub pr_mes: f64,
pub do_mes: f32,
pub sv: u8,
pub mes_qi: i8,
pub cno: i8,
pub lli: u8,

                          }

                          impl WrappedTypesupport for RxmRAWSV { 

            type CStruct = ublox_msgs__msg__RxmRAWSV; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__RxmRAWSV() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__RxmRAWSV {

                unsafe { ublox_msgs__msg__RxmRAWSV__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__RxmRAWSV) -> () {

                unsafe { ublox_msgs__msg__RxmRAWSV__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> RxmRAWSV {
  RxmRAWSV {
cp_mes: msg.cp_mes,
pr_mes: msg.pr_mes,
do_mes: msg.do_mes,
sv: msg.sv,
mes_qi: msg.mes_qi,
cno: msg.cno,
lli: msg.lli,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.cp_mes = self.cp_mes;
msg.pr_mes = self.pr_mes;
msg.do_mes = self.do_mes;
msg.sv = self.sv;
msg.mes_qi = self.mes_qi;
msg.cno = self.cno;
msg.lli = self.lli;
}



        }


                          
                          impl Default for RxmRAWSV {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<RxmRAWSV>::new();
                                  RxmRAWSV::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct RxmRAWX {

                              pub rcv_tow: f64,
pub week: u16,
pub leap_s: i8,
pub num_meas: u8,
pub rec_stat: u8,
pub version: u8,
pub reserved1: Vec<u8>,
pub meas: Vec<ublox_msgs::msg::RxmRAWXMeas>,

                          }

                          impl WrappedTypesupport for RxmRAWX { 

            type CStruct = ublox_msgs__msg__RxmRAWX; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__RxmRAWX() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__RxmRAWX {

                unsafe { ublox_msgs__msg__RxmRAWX__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__RxmRAWX) -> () {

                unsafe { ublox_msgs__msg__RxmRAWX__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> RxmRAWX {
  RxmRAWX {
rcv_tow: msg.rcv_tow,
week: msg.week,
leap_s: msg.leap_s,
num_meas: msg.num_meas,
rec_stat: msg.rec_stat,
version: msg.version,
// is_upper_bound_: false
// member.array_size_ : 2
reserved1: msg.reserved1.to_vec(),
// is_upper_bound_: false
// member.array_size_ : 0
meas : {
let mut temp = Vec::with_capacity(msg.meas.size);
let slice = unsafe { std::slice::from_raw_parts(msg.meas.data, msg.meas.size)};
for s in slice { temp.push(ublox_msgs::msg::RxmRAWXMeas::from_native(s)); }
temp },
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.rcv_tow = self.rcv_tow;
msg.week = self.week;
msg.leap_s = self.leap_s;
msg.num_meas = self.num_meas;
msg.rec_stat = self.rec_stat;
msg.version = self.version;
assert_eq!(self.reserved1.len(), 2, "Field {} is fixed size of {}!", "reserved1", 2);
msg.reserved1.copy_from_slice(&self.reserved1[..2]);
unsafe { ublox_msgs__msg__RxmRAWXMeas__Sequence__fini(&mut msg.meas) };
unsafe { ublox_msgs__msg__RxmRAWXMeas__Sequence__init(&mut msg.meas, self.meas.len()) };
let slice = unsafe { std::slice::from_raw_parts_mut(msg.meas.data, msg.meas.size)};
for (t,s) in slice.iter_mut().zip(&self.meas) { s.copy_to_native(t);}
}



        }


                          
                          impl Default for RxmRAWX {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<RxmRAWX>::new();
                                  RxmRAWX::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct RxmRAWXMeas {

                              pub pr_mes: f64,
pub cp_mes: f64,
pub do_mes: f32,
pub gnss_id: u8,
pub sv_id: u8,
pub reserved0: u8,
pub freq_id: u8,
pub locktime: u16,
pub cno: i8,
pub pr_stdev: u8,
pub cp_stdev: u8,
pub do_stdev: u8,
pub trk_stat: u8,
pub reserved1: u8,

                          }

                          impl WrappedTypesupport for RxmRAWXMeas { 

            type CStruct = ublox_msgs__msg__RxmRAWXMeas; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__RxmRAWXMeas() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__RxmRAWXMeas {

                unsafe { ublox_msgs__msg__RxmRAWXMeas__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__RxmRAWXMeas) -> () {

                unsafe { ublox_msgs__msg__RxmRAWXMeas__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> RxmRAWXMeas {
  RxmRAWXMeas {
pr_mes: msg.pr_mes,
cp_mes: msg.cp_mes,
do_mes: msg.do_mes,
gnss_id: msg.gnss_id,
sv_id: msg.sv_id,
reserved0: msg.reserved0,
freq_id: msg.freq_id,
locktime: msg.locktime,
cno: msg.cno,
pr_stdev: msg.pr_stdev,
cp_stdev: msg.cp_stdev,
do_stdev: msg.do_stdev,
trk_stat: msg.trk_stat,
reserved1: msg.reserved1,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.pr_mes = self.pr_mes;
msg.cp_mes = self.cp_mes;
msg.do_mes = self.do_mes;
msg.gnss_id = self.gnss_id;
msg.sv_id = self.sv_id;
msg.reserved0 = self.reserved0;
msg.freq_id = self.freq_id;
msg.locktime = self.locktime;
msg.cno = self.cno;
msg.pr_stdev = self.pr_stdev;
msg.cp_stdev = self.cp_stdev;
msg.do_stdev = self.do_stdev;
msg.trk_stat = self.trk_stat;
msg.reserved1 = self.reserved1;
}



        }


                          
                          impl Default for RxmRAWXMeas {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<RxmRAWXMeas>::new();
                                  RxmRAWXMeas::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct RxmRTCM {

                              pub version: u8,
pub flags: u8,
pub reserved0: Vec<u8>,
pub ref_station: u16,
pub msg_type: u16,

                          }

                          impl WrappedTypesupport for RxmRTCM { 

            type CStruct = ublox_msgs__msg__RxmRTCM; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__RxmRTCM() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__RxmRTCM {

                unsafe { ublox_msgs__msg__RxmRTCM__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__RxmRTCM) -> () {

                unsafe { ublox_msgs__msg__RxmRTCM__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> RxmRTCM {
  RxmRTCM {
version: msg.version,
flags: msg.flags,
// is_upper_bound_: false
// member.array_size_ : 2
reserved0: msg.reserved0.to_vec(),
ref_station: msg.ref_station,
msg_type: msg.msg_type,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.version = self.version;
msg.flags = self.flags;
assert_eq!(self.reserved0.len(), 2, "Field {} is fixed size of {}!", "reserved0", 2);
msg.reserved0.copy_from_slice(&self.reserved0[..2]);
msg.ref_station = self.ref_station;
msg.msg_type = self.msg_type;
}



        }


                          
                          impl Default for RxmRTCM {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<RxmRTCM>::new();
                                  RxmRTCM::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct RxmSFRB {

                              pub chn: u8,
pub svid: u8,
pub dwrd: Vec<u32>,

                          }

                          impl WrappedTypesupport for RxmSFRB { 

            type CStruct = ublox_msgs__msg__RxmSFRB; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__RxmSFRB() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__RxmSFRB {

                unsafe { ublox_msgs__msg__RxmSFRB__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__RxmSFRB) -> () {

                unsafe { ublox_msgs__msg__RxmSFRB__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> RxmSFRB {
  RxmSFRB {
chn: msg.chn,
svid: msg.svid,
// is_upper_bound_: false
// member.array_size_ : 10
dwrd: msg.dwrd.to_vec(),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.chn = self.chn;
msg.svid = self.svid;
assert_eq!(self.dwrd.len(), 10, "Field {} is fixed size of {}!", "dwrd", 10);
msg.dwrd.copy_from_slice(&self.dwrd[..10]);
}



        }


                          
                          impl Default for RxmSFRB {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<RxmSFRB>::new();
                                  RxmSFRB::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct RxmSFRBX {

                              pub gnss_id: u8,
pub sv_id: u8,
pub reserved0: u8,
pub freq_id: u8,
pub num_words: u8,
pub chn: u8,
pub version: u8,
pub reserved1: u8,
pub dwrd: Vec<u32>,

                          }

                          impl WrappedTypesupport for RxmSFRBX { 

            type CStruct = ublox_msgs__msg__RxmSFRBX; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__RxmSFRBX() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__RxmSFRBX {

                unsafe { ublox_msgs__msg__RxmSFRBX__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__RxmSFRBX) -> () {

                unsafe { ublox_msgs__msg__RxmSFRBX__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> RxmSFRBX {
  RxmSFRBX {
gnss_id: msg.gnss_id,
sv_id: msg.sv_id,
reserved0: msg.reserved0,
freq_id: msg.freq_id,
num_words: msg.num_words,
chn: msg.chn,
version: msg.version,
reserved1: msg.reserved1,
// is_upper_bound_: false
// member.array_size_ : 0
dwrd: msg.dwrd.to_vec(),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.gnss_id = self.gnss_id;
msg.sv_id = self.sv_id;
msg.reserved0 = self.reserved0;
msg.freq_id = self.freq_id;
msg.num_words = self.num_words;
msg.chn = self.chn;
msg.version = self.version;
msg.reserved1 = self.reserved1;
msg.dwrd.update(&self.dwrd);
}



        }


                          
                          impl Default for RxmSFRBX {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<RxmSFRBX>::new();
                                  RxmSFRBX::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct RxmSVSI {

                              pub i_tow: i32,
pub week: i16,
pub num_vis: u8,
pub num_sv: u8,
pub sv: Vec<ublox_msgs::msg::RxmSVSISV>,

                          }

                          impl WrappedTypesupport for RxmSVSI { 

            type CStruct = ublox_msgs__msg__RxmSVSI; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__RxmSVSI() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__RxmSVSI {

                unsafe { ublox_msgs__msg__RxmSVSI__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__RxmSVSI) -> () {

                unsafe { ublox_msgs__msg__RxmSVSI__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> RxmSVSI {
  RxmSVSI {
i_tow: msg.i_tow,
week: msg.week,
num_vis: msg.num_vis,
num_sv: msg.num_sv,
// is_upper_bound_: false
// member.array_size_ : 0
sv : {
let mut temp = Vec::with_capacity(msg.sv.size);
let slice = unsafe { std::slice::from_raw_parts(msg.sv.data, msg.sv.size)};
for s in slice { temp.push(ublox_msgs::msg::RxmSVSISV::from_native(s)); }
temp },
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.i_tow = self.i_tow;
msg.week = self.week;
msg.num_vis = self.num_vis;
msg.num_sv = self.num_sv;
unsafe { ublox_msgs__msg__RxmSVSISV__Sequence__fini(&mut msg.sv) };
unsafe { ublox_msgs__msg__RxmSVSISV__Sequence__init(&mut msg.sv, self.sv.len()) };
let slice = unsafe { std::slice::from_raw_parts_mut(msg.sv.data, msg.sv.size)};
for (t,s) in slice.iter_mut().zip(&self.sv) { s.copy_to_native(t);}
}



        }


                          
                          impl Default for RxmSVSI {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<RxmSVSI>::new();
                                  RxmSVSI::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct RxmSVSISV {

                              pub svid: u8,
pub sv_flag: u8,
pub azim: i16,
pub elev: i8,
pub age: u8,

                          }

                          impl WrappedTypesupport for RxmSVSISV { 

            type CStruct = ublox_msgs__msg__RxmSVSISV; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__RxmSVSISV() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__RxmSVSISV {

                unsafe { ublox_msgs__msg__RxmSVSISV__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__RxmSVSISV) -> () {

                unsafe { ublox_msgs__msg__RxmSVSISV__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> RxmSVSISV {
  RxmSVSISV {
svid: msg.svid,
sv_flag: msg.sv_flag,
azim: msg.azim,
elev: msg.elev,
age: msg.age,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.svid = self.svid;
msg.sv_flag = self.sv_flag;
msg.azim = self.azim;
msg.elev = self.elev;
msg.age = self.age;
}



        }


                          
                          impl Default for RxmSVSISV {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<RxmSVSISV>::new();
                                  RxmSVSISV::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct TimTM2 {

                              pub ch: u8,
pub flags: u8,
pub rising_edge_count: u16,
pub wn_r: u16,
pub wn_f: u16,
pub tow_ms_r: u32,
pub tow_sub_ms_r: u32,
pub tow_ms_f: u32,
pub tow_sub_ms_f: u32,
pub acc_est: u32,

                          }

                          impl WrappedTypesupport for TimTM2 { 

            type CStruct = ublox_msgs__msg__TimTM2; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__TimTM2() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__TimTM2 {

                unsafe { ublox_msgs__msg__TimTM2__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__TimTM2) -> () {

                unsafe { ublox_msgs__msg__TimTM2__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> TimTM2 {
  TimTM2 {
ch: msg.ch,
flags: msg.flags,
rising_edge_count: msg.rising_edge_count,
wn_r: msg.wn_r,
wn_f: msg.wn_f,
tow_ms_r: msg.tow_ms_r,
tow_sub_ms_r: msg.tow_sub_ms_r,
tow_ms_f: msg.tow_ms_f,
tow_sub_ms_f: msg.tow_sub_ms_f,
acc_est: msg.acc_est,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.ch = self.ch;
msg.flags = self.flags;
msg.rising_edge_count = self.rising_edge_count;
msg.wn_r = self.wn_r;
msg.wn_f = self.wn_f;
msg.tow_ms_r = self.tow_ms_r;
msg.tow_sub_ms_r = self.tow_sub_ms_r;
msg.tow_ms_f = self.tow_ms_f;
msg.tow_sub_ms_f = self.tow_sub_ms_f;
msg.acc_est = self.acc_est;
}



        }


                          
                          impl Default for TimTM2 {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<TimTM2>::new();
                                  TimTM2::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct UpdSOS {

                              pub cmd: u8,
pub reserved1: Vec<u8>,

                          }

                          impl WrappedTypesupport for UpdSOS { 

            type CStruct = ublox_msgs__msg__UpdSOS; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__UpdSOS() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__UpdSOS {

                unsafe { ublox_msgs__msg__UpdSOS__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__UpdSOS) -> () {

                unsafe { ublox_msgs__msg__UpdSOS__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> UpdSOS {
  UpdSOS {
cmd: msg.cmd,
// is_upper_bound_: false
// member.array_size_ : 3
reserved1: msg.reserved1.to_vec(),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.cmd = self.cmd;
assert_eq!(self.reserved1.len(), 3, "Field {} is fixed size of {}!", "reserved1", 3);
msg.reserved1.copy_from_slice(&self.reserved1[..3]);
}



        }


                          
                          impl Default for UpdSOS {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<UpdSOS>::new();
                                  UpdSOS::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct UpdSOSAck {

                              pub cmd: u8,
pub reserved0: Vec<u8>,
pub response: u8,
pub reserved1: Vec<u8>,

                          }

                          impl WrappedTypesupport for UpdSOSAck { 

            type CStruct = ublox_msgs__msg__UpdSOSAck; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__UpdSOSAck() }
            }

            fn create_msg() -> *mut ublox_msgs__msg__UpdSOSAck {

                unsafe { ublox_msgs__msg__UpdSOSAck__create() }

            }

            fn destroy_msg(msg: *mut ublox_msgs__msg__UpdSOSAck) -> () {

                unsafe { ublox_msgs__msg__UpdSOSAck__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> UpdSOSAck {
  UpdSOSAck {
cmd: msg.cmd,
// is_upper_bound_: false
// member.array_size_ : 3
reserved0: msg.reserved0.to_vec(),
response: msg.response,
// is_upper_bound_: false
// member.array_size_ : 3
reserved1: msg.reserved1.to_vec(),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.cmd = self.cmd;
assert_eq!(self.reserved0.len(), 3, "Field {} is fixed size of {}!", "reserved0", 3);
msg.reserved0.copy_from_slice(&self.reserved0[..3]);
msg.response = self.response;
assert_eq!(self.reserved1.len(), 3, "Field {} is fixed size of {}!", "reserved1", 3);
msg.reserved1.copy_from_slice(&self.reserved1[..3]);
}



        }


                          
                          impl Default for UpdSOSAck {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<UpdSOSAck>::new();
                                  UpdSOSAck::from_native(&msg_native)
                              }
                          }
             


                      }
