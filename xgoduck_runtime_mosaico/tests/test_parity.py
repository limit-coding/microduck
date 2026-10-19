"""Run compiled firmware C against the original Python/ONNX implementation, no hardware."""
import argparse
import ctypes as C
from pathlib import Path
import struct
import sys
import unittest
import numpy as np
import onnxruntime as ort

F3=C.c_float*3; F14=C.c_float*14; F15=C.c_float*15
class State(C.Structure):
    _fields_=[('acc',F3),('gyro',F3),('angles',F15),('vels',F15),('gravity',F3),('gyro_rad',F3),('battery',C.c_uint8)]
class Filter(C.Structure):
    _fields_=[('initialized',C.c_bool),('gravity',F3),('gyro',F3),('vel',F15)]
class OutputFilter(C.Structure):
    _fields_=[('initialized',C.c_bool),('value',F15)]
class Estimator(C.Structure):
    _fields_=[('initialized',C.c_bool),('gravity',F3)]
class ImuSnapshot(C.Structure):
    _fields_=[('ready',C.c_bool),('estimate_valid',C.c_bool),('chip_id',C.c_uint8),('error',C.c_int),
             ('acc',C.c_int16*3),('gyro',C.c_int16*3),('acc_g',F3),('gyro_rad',F3),('gravity',F3),
             ('samples',C.c_uint32),('errors',C.c_uint32),('sensor_time',C.c_uint32),('sample_us',C.c_int64)]
class Parser(C.Structure):
    _fields_=[('bytes',C.c_uint8*254),('used',C.c_size_t)]

class Parity(unittest.TestCase):
    def test_model(self):
        rng=np.random.default_rng(31); worst=0.
        for i in range(150):
            obs=(rng.standard_normal(61)*[0,0.1,1,3,10][i%5]).astype(np.float32)
            expected=session.run(None,{'obs':obs[None]})[0].ravel()
            out=F14();scratch=(C.c_float*1024)()
            self.assertTrue(lib.policy_run((C.c_float*61)(*obs),out,scratch))
            error=float(np.max(np.abs(np.array(out)-expected)));worst=max(worst,error)
            np.testing.assert_allclose(out,expected,rtol=2e-4,atol=2e-4)
        print(f'150 ONNX/C vectors: maximum absolute action error = {worst:.9g} rad')
        for index in [0,17,60]:
            for value in [np.nan,np.inf,-np.inf]:
                obs=np.zeros(61,np.float32);obs[index]=value
                self.assertFalse(lib.policy_run((C.c_float*61)(*obs),F14(),(C.c_float*1024)()))

    def test_relocated_weights(self):
        import json
        manifest=json.loads((Path(__file__).parents[1]/'main/policy_manifest.json').read_text())
        count=manifest['weight_floats']
        original=(C.c_float*count).in_dll(lib,'policy_weights')
        copied=(C.c_float*count).from_buffer_copy(original)
        rng=np.random.default_rng(1416)
        for k in range(40):
            obs=(C.c_float*61)(*rng.normal(0,1,61));out=F14();reference=F14();scratch=(C.c_float*1024)()
            self.assertTrue(lib.policy_run_weights(copied,obs,out,scratch))
            self.assertTrue(lib.policy_run_reference(original,obs,reference,scratch))
            np.testing.assert_allclose(out,reference,atol=2e-4,rtol=2e-4)
        # The explicit pointer must govern every layer, including the output bias.
        obs=(C.c_float*61)();before=F14();after=F14();scratch=(C.c_float*1024)()
        self.assertTrue(lib.policy_run_weights(copied,obs,before,scratch))
        copied[count-1]+=0.25
        self.assertTrue(lib.policy_run_weights(copied,obs,after,scratch))
        self.assertAlmostEqual(after[13]-before[13],0.25,places=5)
        copied[61]=0  # Invalid normalizer divisor must not reach the motors.
        self.assertFalse(lib.policy_run_weights(copied,obs,after,scratch))

    def test_default_targets(self):
        targets=F15();lib.duck_targets(F14(),targets)
        np.testing.assert_array_equal(targets,[0,-5,-20,0,20,0,5,20,0,-20,20,20,0,0,0])

    def test_action_output_filter(self):
        f=OutputFilter()
        first=np.arange(15,dtype=np.float32)-7
        current=F15(*first);lib.duck_filter_targets(C.byref(f),current)
        np.testing.assert_array_equal(current,first)
        second=first+10
        current=F15(*second);lib.duck_filter_targets(C.byref(f),current)
        expected=0.3*first+0.7*second
        np.testing.assert_allclose(current,expected,atol=1e-6)
        third=first-4
        current=F15(*third);lib.duck_filter_targets(C.byref(f),current)
        np.testing.assert_allclose(current,0.3*expected+0.7*third,atol=1e-6)
        f.initialized=False
        current=F15(*second);lib.duck_filter_targets(C.byref(f),current)
        np.testing.assert_array_equal(current,second)

    def test_mosaico_raw(self):
        # Signed extrema, little-endian order, XYZ order and 24-bit clock; no transformations.
        for values,ticks in [([-32768,-1,32767,1234,-2345,0],0xffffff),([1,2,3,-4,-5,-6],0x123456)]:
            packet=struct.pack('<6h',*values)+ticks.to_bytes(3,'little')
            acc=(C.c_int16*3)();gyro=(C.c_int16*3)();clock=C.c_uint32()
            lib.mosaico_imu_decode((C.c_uint8*15).from_buffer_copy(packet),acc,gyro,C.byref(clock))
            self.assertEqual(list(acc)+list(gyro),values)
            self.assertEqual(clock.value,ticks)

    def test_mosaico_attitude(self):
        f=Estimator();acc_g=F3();gyro_rad=F3();gravity=F3()
        def step(a,w,dt=.01):
            return lib.mosaico_estimate(C.byref(f),(C.c_int16*3)(*a),(C.c_int16*3)(*w),C.c_float(dt),acc_g,gyro_rad,gravity)
        self.assertFalse(step([0,0,0],[0,0,0]))
        self.assertTrue(step([8192,0,0],[-32768,16384,-8192]))
        np.testing.assert_allclose(acc_g,[0,0,1],atol=1e-6)
        np.testing.assert_allclose(gyro_rad,np.deg2rad([500,-1000,2000]),rtol=2e-7)
        np.testing.assert_allclose(gravity,[0,0,1],atol=1e-6)
        # Known constant body roll, with acceleration correction disabled by zero magnitude.
        for k in range(100): self.assertTrue(step([0,0,0],[0,0,1475]))
        angle=np.deg2rad(1475*2000/32768)
        np.testing.assert_allclose(gravity,[0,np.sin(angle),np.cos(angle)],atol=2e-5)
        self.assertAlmostEqual(np.linalg.norm(gravity),1,places=6)
        # Excess linear acceleration must not pull gravity toward the acceleration axis.
        saved=np.array(gravity)
        for k in range(100): self.assertTrue(step([0,32767,0],[0,0,0]))
        np.testing.assert_allclose(gravity,saved,atol=1e-6)
        # A real stable direction corrects integrated tilt over time.
        for k in range(400): self.assertTrue(step([0,0,8192],[0,0,0]))
        np.testing.assert_allclose(gravity,[1,0,0],atol=1e-5)
        self.assertTrue(step([0,8192,0],[0,0,0],.2))
        np.testing.assert_allclose(gravity,[0,1,0],atol=1e-6)
        self.assertFalse(step([0,0,0],[0,0,0],.2))
        self.assertFalse(step([8192,0,0],[0,0,0],float('nan')))

    def test_mosaico_freshness(self):
        st=ImuSnapshot(ready=True,estimate_valid=True,samples=5,sample_us=500000)
        self.assertTrue(lib.mosaico_imu_fresh(C.byref(st),C.c_int64(600000)))
        self.assertFalse(lib.mosaico_imu_fresh(C.byref(st),C.c_int64(600001)))
        self.assertFalse(lib.mosaico_imu_fresh(C.byref(st),C.c_int64(499999)))
        for field,bad in [('ready',False),('estimate_valid',False),('samples',0),('error',1)]:
            old=getattr(st,field);setattr(st,field,bad)
            self.assertFalse(lib.mosaico_imu_fresh(C.byref(st),C.c_int64(500000)))
            setattr(st,field,old)

    def test_peer_imu_required(self):
        raw=bytearray(self.packet()[2:-2]);state=State()
        self.assertTrue(lib.duck_decode((C.c_uint8*250).from_buffer_copy(raw),C.byref(state)))
        struct.pack_into('<9f',raw,20,*([float('nan')]*9))
        self.assertFalse(lib.duck_decode((C.c_uint8*250).from_buffer_copy(raw),C.byref(state)))

    def packet(self,k=0):
        p=bytearray(250);p[0]=80
        struct.pack_into('<9f',p,20,0,0,0,0.2,9.8,0.1,1+k,-2,3)
        struct.pack_into('<15f',p,56,*[i*2.5+k for i in range(15)])
        struct.pack_into('<15f',p,120,*[i*3.25-k for i in range(15)])
        return b'\x55\0'+p+b'\0\xaa'

    def test_stream_parser(self):
        good=self.packet();bad=bytearray(good);bad[-1]=0
        nan=bytearray(good);struct.pack_into('<f',nan,58,float('nan'))
        stream=b'DUCK_IP=192.168.1.2\r\n\x55'+bytes(bad)+good[:123]+good+bytes(nan)+good+good
        p=Parser();c_states=[];st=State()
        for b in stream:
            if lib.duck_parse_byte(C.byref(p),b,C.byref(st)): c_states.append(bytes(st))
        expected=duck.Parser().feed_bytes(stream)
        self.assertEqual(len(c_states),len(expected));self.assertGreaterEqual(len(c_states),3)
        for raw,py in zip(c_states,expected):
            c=State.from_buffer_copy(raw)
            np.testing.assert_array_equal(c.angles,py.angles);np.testing.assert_array_equal(c.vels,py.vels)

    def test_filter_observation_targets_frames(self):
        cf=Filter();pf=duck.ImuFilter();prev=np.zeros(14,np.float32)
        for k in range(200):
            packet=self.packet(k/20);payload=packet[2:-2];st=State()
            self.assertTrue(lib.duck_decode((C.c_uint8*250).from_buffer_copy(payload),C.byref(st)))
            py=duck.decode_payload(payload);dt=[.01,.005,.025,.12,.001][k%5]
            lib.duck_filter(C.byref(cf),C.byref(st),C.c_float(dt));pf.update(py,dt)
            np.testing.assert_allclose(st.vels,py.vels,atol=1e-5)
            np.testing.assert_allclose(st.gravity,py.grav,atol=2e-6)
            np.testing.assert_allclose(st.gyro_rad,py.gyro_rad,atol=2e-6)
            twist=np.array([.2,-.1,.3],np.float32);head=np.array([.1,.2,-.3,.4],np.float32)
            obs=(C.c_float*61)();lib.duck_observation(C.byref(st),st.gyro_rad,st.gravity,F14(*prev),F3(*twist),(C.c_float*4)(*head),obs)
            expected=infer.build_obs(py,prev,twist,head)
            np.testing.assert_allclose(obs,expected,atol=2e-6)
            action=F14();self.assertTrue(lib.policy_run(obs,action,(C.c_float*1024)()))
            reference=session.run(None,{'obs':expected[None]})[0].ravel()
            np.testing.assert_allclose(action,reference,atol=2e-4,rtol=2e-4)
            prev=np.array(action)
            targets=F15();lib.duck_targets(action,targets)
            expected_targets=infer.action_to_deg(prev)+np.array([0,0,3,0,-3,0,0,-3,0,3,0,0,0,0,0],np.float32)
            np.testing.assert_allclose(targets,expected_targets,atol=5e-5)
            frame=(C.c_uint8*83)();self.assertEqual(lib.duck_angles_frame(targets,frame),83)
            p=b''.join(bytes([i])+struct.pack('<f',v) for i,v in zip(duck.MOTOR_IDS,targets))
            self.assertEqual(bytes(frame),duck.build_write(40,p))
        for major,value in [(5,5),(3,1)]:
            frame=(C.c_uint8*10)();p=(C.c_uint8*2)(1,value)
            self.assertEqual(lib.duck_write(major,p,2,frame),10)
            self.assertEqual(bytes(frame),duck.build_write(major,bytes(p)))

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--reference',type=Path,required=True);args=ap.parse_args()
    sys.path.insert(0,str(args.reference));import duck;import infer
    session=ort.InferenceSession(str(args.reference/'xgoduck_walk.onnx'),providers=['CPUExecutionProvider'])
    lib=C.CDLL(str(Path(__file__).with_name('duck_host.dll')))
    lib.mosaico_estimate.restype=C.c_bool;lib.mosaico_imu_fresh.restype=C.c_bool
    lib.policy_run.restype=C.c_bool;lib.policy_run_weights.restype=C.c_bool;lib.policy_run_reference.restype=C.c_bool
    lib.duck_parse_byte.restype=C.c_bool;lib.duck_decode.restype=C.c_bool
    lib.duck_angles_frame.restype=C.c_size_t;lib.duck_write.restype=C.c_size_t
    unittest.main(argv=[sys.argv[0]],verbosity=2)
