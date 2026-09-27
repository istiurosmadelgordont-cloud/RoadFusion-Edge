import json
import queue
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from app import App, check_network, list_local_ips, network_parameters
from protocol import Sender
from test_sender import FakeSocket


class NetworkTests(unittest.TestCase):
    def test_addresses_and_mac_normalized(self):
        for mac in ('02:11:22:33:44:66', '021122334466', '02-11-22-33-44-66'):
            self.assertEqual(network_parameters(' 10.20.1.2 ', '10.20.2.3', mac),
                             ('10.20.1.2', '10.20.2.3', '02-11-22-33-44-66'))

    def test_invalid_values_rejected_before_powershell(self):
        cases = [('not-an-ip', '10.1.1.2', '021122334466'),
                 ('10.1.1.2', '10.1.1.2', '021122334466'),
                 ('10.1.1.1', "10.1.1.2';Write-Host test", '021122334466'),
                 ('10.1.1.1', '224.0.0.1', '021122334466'),
                 ('10.1.1.1', '10.1.1.2', 'FF-FF-FF-FF-FF-FF'),
                 ('10.1.1.1', '10.1.1.2', '000000000000'),
                 ('10.1.1.1', '10.1.1.2', '02-11-22-33-44')]
        with patch('app.subprocess.run') as run:
            for args in cases:
                with self.subTest(args=args), self.assertRaises(ValueError):
                    check_network(*args)
            run.assert_not_called()

    def test_custom_subnet_and_mac_preflight(self):
        data = dict(Prefix=16, Status='Up', Speed='1 Gbps', SpeedBps=10**9,
                    Mtu=1500, Neighbors=[dict(LinkLayerAddress='02:11:22:33:44:66', State='Permanent')])
        with patch('app.subprocess.run') as run:
            def response(*args, **kwargs):
                return SimpleNamespace(returncode=0, stdout=json.dumps(data).encode())
            run.side_effect = response
            self.assertIn('静态 ARP 正确', check_network('10.20.1.2', '10.20.2.3', '021122334466', 1472))
            script = run.call_args.args[0][-1]
            self.assertIn("-IPAddress '10.20.2.3'", script)
            self.assertNotIn('192.168.1.10', script)
            for destination in ('10.21.1.2', '10.20.255.255', '10.20.0.0'):
                with self.assertRaises(ValueError):
                    check_network('10.20.1.2', destination, '021122334466')
            data['Mtu'] = 1400
            with self.assertRaises(ValueError):
                check_network('10.20.1.2', '10.20.2.3', '021122334466', 1472)
            data['Mtu'] = 1500
            data['Neighbors'][0]['State'] = 'Reachable'
            with self.assertRaisesRegex(ValueError, '-TargetIP 10.20.2.3 -TargetMAC 02-11-22-33-44-66'):
                check_network('10.20.1.2', '10.20.2.3', '021122334466')

    def test_adapter_list_zero_one_many(self):
        with patch('app.subprocess.run') as run:
            for addresses in ([], ['10.0.0.2'], ['192.168.1.102', '10.0.0.2']):
                run.return_value = SimpleNamespace(returncode=0, stdout=json.dumps(addresses).encode())
                self.assertEqual(list_local_ips(), sorted(addresses))

    def test_sender_binds_host_and_sends_every_packet_to_custom_target(self):
        sock = Mock()
        sock.sendto.side_effect = lambda data, addr: len(data)
        with patch('protocol.socket.socket', return_value=sock):
            sender = Sender('10.20.1.2', destination='10.20.2.3', size=(2, 2), wait=lambda _: None)
            sender.send_frame(bytes(8))
            sock.bind.assert_called_once_with(('10.20.1.2', 0))
            self.assertEqual(len(sock.sendto.call_args_list), 3)
            for call in sock.sendto.call_args_list:
                self.assertEqual(call.args[1], ('10.20.2.3', 1234))

    def test_gui_worker_passes_selected_network_parameters(self):
        app = App.__new__(App)
        app.stop = threading.Event()
        app.events = queue.Queue()
        app.preview = queue.Queue(maxsize=1)
        class StopSocket(FakeSocket):
            def sendto(self, payload, address):
                app.stop.set()
                return super().sendto(payload, address)
        sock = StopSocket()
        def make_sender(*args, **kwargs):
            return Sender(*args, **kwargs, sock=sock, wait=lambda _: None)
        config = ('10.20.1.2', 10, 120, '彩条测试', '', True,
                  None, 30, (2, 2), '等比留黑', 2, 256, 20,
                  '10.20.2.3', '02-11-22-33-44-66')
        with patch('app.check_network', return_value='test') as check, \
                patch('app.Sender', side_effect=make_sender):
            app.run(config)
            check.assert_called_once_with('10.20.1.2', '10.20.2.3', '02-11-22-33-44-66', 256)
        self.assertEqual(len(sock.sent), 3)
        self.assertTrue(all(addr == ('10.20.2.3', 1234) for _, addr in sock.sent))
        self.assertFalse(any(kind in ('error', 'setup_error') for kind, _ in app.events.queue))


if __name__ == '__main__':
    unittest.main()
