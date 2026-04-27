import pytest
from fakes import FakeSerial

@pytest.fixture
def fake_serial(monkeypatch):
    import lac1

    container = {'instance': None}

    def factory(*args, **kwargs):
        fake = FakeSerial(*args, **kwargs)
        container['instance'] = fake
        return fake

    monkeypatch.setattr(
        lac1.serial,
        "Serial",
        factory

    )
    return container

from lac1 import LAC1
def test_init(fake_serial):
    controller = LAC1(port='COM_TEST', baudRate=9600)

    fake = fake_serial['instance']
    assert fake.port == 'COM_TEST'
    assert fake.written[0] == b'EF\r'

def test_sendcmds(fake_serial):
    controller = LAC1(port='COM_TEST', baudRate=9600)
    controller.sendcmds('AAAA')

    fake = fake_serial['instance']
    assert fake.written[-1] == b'AAAA\r'
    
def test_go(fake_serial):
    controller = LAC1(port='COM_TEST', baudRate=9600)
    controller.go()

    fake = fake_serial['instance']
    assert fake.written[-1] == b'GO\r'

def test_set_max_velocity(fake_serial):
    controller = LAC1(port='COM_TEST', baudRate=9600)
    controller.set_max_velocity(100)

    fake = fake_serial['instance']
    assert fake.written[-1] == b'SV1310720\r'

def test_get_position_enc(fake_serial):
    controller = LAC1(port='COM_TEST', baudRate=9600)

    fake = fake_serial['instance']
    fake.queue_response(b'1000\r')
    pos = controller.get_position_enc()
    assert pos == 1000

