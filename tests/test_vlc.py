import pytest
from .fakes import FakeSerial
from smac_python.lac1 import Actuator


@pytest.fixture
def fake_serial(monkeypatch):
    import smac_python.lac1 as lac1

    container = {'instance': None}

    def factory(*args, **kwargs):
        fake = FakeSerial(*args, **kwargs)
        container['instance'] = fake
        return fake

    monkeypatch.setattr(lac1.serial, "Serial", factory)
    return container


from smac_python.vlc import VLC


def test_vlc_init(fake_serial):
    controller = VLC(port='COM_TEST', baudRate=9600)

    fake = fake_serial['instance']
    assert fake.port == 'COM_TEST'
    assert fake.written[0] == b'EF\r'


def test_it2_ex1(fake_serial):
    # Example 1 from VLCI-X1 manual
    controller = VLC(port='COM_TEST', baudRate=9600)
    controller.set_i2t_protection(
        nominal_current='1 A', peak_current='3 A', time_period='1 s'
    )

    fake = fake_serial['instance']
    assert fake.written[-1] == b'AL177,WW610,AL249571000,WL612,AL0,WB620\r'


def test_it2_ex2(fake_serial):
    # Example 2 from VLCI-X1 manual, but i2tINTRVL was only increased to 2 ms
    controller = VLC(port='COM_TEST', baudRate=9600)
    controller.set_i2t_protection(
        nominal_current='1 A', peak_current='3 A', time_period='10 s'
    )

    fake = fake_serial['instance']
    assert fake.written[-1] == b'AL177,WW610,AL1247855000,WL612,AL1,WB620\r'


def test_phase_offset_calibration(fake_serial):
    controller = VLC(port='COM_TEST', baudRate=9600)
    controller.phase_offset_calibration()

    fake = fake_serial['instance']
    assert fake.written[-4] == b'MD300,AL0,AR4,AR5,MF,WA50\r'
    assert (
        fake.written[-3] == b'MD301,RW622,AA@4,AR4,RW624,AA@5,AR5,WA1,RP999\r'
    )
    assert (
        fake.written[-2]
        == b'MD302,AL0,AR1,AL2048000,AS@4,AD1000,AR4,WW606,AL0,AR1,AL2048000,AS@5,AD1000,AR5,WW608,MG"Offset A/B: ":4:N,MG"/":5\r'
    )
    assert fake.written[-1] == b'MS300\r'


def test_phase_motors(fake_serial, monkeypatch):
    monkeypatch.setattr("builtins.input", lambda _: "\r")

    controller = VLC(port='COM_TEST', baudRate=9600)
    controller.phase_motors()

    fake = fake_serial['instance']
    assert (
        fake.written[-10] == b'MD200,PH1,MF,EC0,SP0,PM,SQ32767,QM0,MN,SQ9502\r'
    )
    assert (
        fake.written[-9]
        == b'MD201,AL23900,AR4,RL494,AR3,SP16383,WA400,RL494,AS@3,AR5,IG10755,IB13145,MJ208,NO,MC207\r'
    )
    assert (
        fake.written[-8]
        == b'MD202,AL35850,AR4,RL494,AR3,SP32767,WA400,RL494,AS@3,AR5,IG10755,IB13145,MJ208,NO,MC207\r'
    )
    assert (
        fake.written[-7]
        == b'MD203,AL47800,AR4,RL494,AR3,SP49150,WA400,RL494,AS@3,AR5,IG10755,IB13145,MJ208,NO,MC207\r'
    )
    assert (
        fake.written[-6]
        == b'MD204,AL35850,AR4,RL494,AR3,SP32767,WA400,RL494,AS@3,AM-1,AR5,IG10755,IB13145,MJ208,NO,MC207\r'
    )
    assert (
        fake.written[-5]
        == b'MD205,AL23900,AR4,RL494,AR3,SP16383,WA400,RL494,AS@3,AM-1,AR5,IG10755,IB13145,MJ208,NO,MC207\r'
    )
    assert (
        fake.written[-4]
        == b'MD206,AL11950,AR4,RL494,AR3,SP0,WA400,RL494,AS@3,AM-1,AR5,IG10755,IB13145,MJ208,NO,MC207,SQ0,MG"Phasing NOK",EP \r'
    )
    assert (
        fake.written[-3]
        == b'MD207,NO,MG"Ideal step [enc cnts]= 11950. Actual step= ":5,RC \r'
    )
    assert (
        fake.written[-2]
        == b'MD208,RA4,DA@0,EC47800,SQ0,MF,MG"Phasing Succesfull, stepsize (11950)= ":5\r'
    )
    assert fake.written[-1] == b'MS200\r'

def test_parameter_save(fake_serial):
    controller = VLC(port='COM_TEST', baudRate=9600)
    controller.parameter_save()

    fake = fake_serial['instance']
    assert fake.written[-1] == b'PS\r'

def test_max_force(fake_serial):
    controller = VLC(port='COM_TEST', baudRate=9600)
    controller.set_max_force('10 N')

    fake = fake_serial['instance']
    assert fake.written[-1] == b'SQ3640\r'

def test_max_force_voltage(fake_serial):
    controller = VLC(port='COM_TEST', baudRate=9600)
    controller.set_max_force('10 N', mode='voltage')

    fake = fake_serial['instance']
    assert fake.written[-1] == b'SQ3640\r'

def test_max_force_current(fake_serial):
    controller = VLC(port='COM_TEST', baudRate=9600)
    controller.set_max_force('10 N', mode='current')

    fake = fake_serial['instance']
    assert fake.written[-1] == b'SQ43\r'

def test_move_to_force(fake_serial):
    actuator = Actuator(force_constant='41 N/A')
    controller = VLC(port='COM_TEST', baudRate=9600, actuator=actuator)
    controller.move_to_force('10 N')

    fake = fake_serial['instance']
    assert fake.written[-2] == b'SQ3640,VM,MN,GO\r'
    assert fake.written[-1] == b'WA25,RW548,IG43,NO,EP,RP1000\r'

def test_move_to_force_hold(fake_serial):
    actuator = Actuator(force_constant='41 N/A')
    controller = VLC(port='COM_TEST', baudRate=9600, actuator=actuator)
    controller.move_to_force('10 N', hold=True)

    fake = fake_serial['instance']
    assert fake.written[-3] == b'SQ3640,VM,MN,GO\r'
    assert fake.written[-2] == b'WA25,RW548,IG43,NO,EP,RP1000\r'
    assert fake.written[-1] == b'SC8000,QM1,SQ43\r'

