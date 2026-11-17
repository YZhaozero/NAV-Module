from datetime import datetime,timezone
import json
import unittest
from nav_rtk.parser.checksum import calculate_nmea_checksum
from nav_rtk.bridge.observation import ObservationBuilder


def sentence(body):return '$'+body+'*'+calculate_nmea_checksum(body)

class ObservationTest(unittest.TestCase):
    def setUp(self):
        self.builder=ObservationBuilder()
        self.stamp=datetime(2026,10,4,12,0,0,tzinfo=timezone.utc).timestamp()
        self.builder.feed(sentence('GNRMC,120000.00,A,3954.0000,N,11618.0000,E,0,0,041026,,,A'),self.stamp)
    def gga(self,quality=4,time='120000.00'):
        return sentence(f'GNGGA,{time},3954.0000,N,11618.0000,E,{quality},20,0.7,50.0,M,-10.0,M,1.2,01')
    def gst(self,time='120000.00',values='0.10,0.20,0.30'):
        return sentence(f'GNGST,{time},1,1,1,0,{values}')
    def test_epoch_and_altitude_and_unknown_covariance(self):
        result=self.builder.feed(self.gga(),self.stamp+.05)
        self.assertEqual(result['stamp'],self.stamp)
        self.assertEqual(result['altitude_ellipsoid'],40)
        self.assertEqual(result['altitude_msl'],50)
        self.assertEqual(result['differential_age_s'],1.2)
        self.assertIsNone(result['covariance'])
        self.assertIsNone(result['horizontal_std_m'])
    def test_gst_same_epoch_only(self):
        self.builder.feed(sentence('GNGST,120000.00,1,1,1,0,0.10,0.20,0.30'),self.stamp+.01)
        result=self.builder.feed(self.gga(),self.stamp+.05)
        self.assertAlmostEqual(result['covariance'][0],.04)
        self.assertAlmostEqual(result['horizontal_std_m'],.2)
        result=self.builder.feed(self.gga(time='120001.00'),self.stamp+1)
        self.assertIsNone(result['covariance'])
    def test_six_epochs_get_same_accuracy_in_either_sentence_order(self):
        for gst_first in (True,False):
            builder=ObservationBuilder()
            accurate=[]
            for index in range(6):
                epoch=f'120000.{index*10:02}'
                messages=[self.gst(epoch),self.gga(time=epoch)]
                if not gst_first:messages.reverse()
                for offset,message in enumerate(messages):
                    result=builder.feed(message,self.stamp+index*.1+offset*.01)
                    if result and result['covariance'] is not None:accurate.append(result)
            with self.subTest(gst_first=gst_first):
                self.assertEqual(len(accurate),6)
                self.assertTrue(all(abs(item['horizontal_std_m']-.2)<1e-10 for item in accurate))
    def test_late_gst_updates_once_without_renewing_original_fix(self):
        original=self.builder.feed(self.gga(),self.stamp+.05)
        update=self.builder.feed(self.gst(time='120000.000'),self.stamp+.15)
        self.assertIsNotNone(update)
        self.assertTrue(update['covariance_update'])
        self.assertEqual(update['stamp'],original['stamp'])
        self.assertEqual(update['received_at'],original['received_at'])
        self.assertEqual(update['latitude'],original['latitude'])
        self.assertIsNone(original['covariance'])
        self.assertAlmostEqual(update['horizontal_std_m'],.2)
        self.assertIsNone(self.builder.feed(self.gst(),self.stamp+.2))
    def test_adjacent_20hz_epochs_are_never_paired(self):
        for gst_first in (True,False):
            for epoch in ('115959.95','120000.05','120000.04'):
                builder=ObservationBuilder()
                if gst_first:
                    builder.feed(self.gst(epoch),self.stamp)
                    self.assertIsNone(builder.feed(self.gga(),self.stamp+.01)['covariance'])
                else:
                    builder.feed(self.gga(),self.stamp)
                    self.assertIsNone(builder.feed(self.gst(epoch),self.stamp+.01))
    def test_late_gst_requires_known_epoch_and_fresh_forward_receipt(self):
        for delay in (-.01,1.,2.):
            builder=ObservationBuilder()
            builder.feed(self.gga(),self.stamp)
            self.assertIsNone(builder.feed(self.gst(),self.stamp+delay))
            builder=ObservationBuilder()
            builder.feed(self.gst(),self.stamp)
            self.assertIsNone(builder.feed(self.gga(),self.stamp+delay)['covariance'])
        self.builder.feed(self.gga(),self.stamp)
        self.builder.feed(self.gga(time='120000.10'),self.stamp+.1)
        first_update=self.builder.feed(self.gst(),self.stamp+.15)
        self.assertIsNotNone(first_update)
        self.assertEqual(first_update['stamp'],self.stamp)
        self.assertIsNotNone(self.builder.feed(self.gst('120000.10'),self.stamp+.2))
    def test_invalid_gst_cannot_supply_accuracy_or_position(self):
        for values in ('0,0.2,0.3','-1,0.2,0.3','nan,0.2,0.3','inf,0.2,0.3','0.1,,0.3'):
            builder=ObservationBuilder()
            builder.feed(self.gga(),self.stamp)
            self.assertIsNone(builder.feed(self.gst(values=values),self.stamp+.1))
            self.assertIsNone(builder.feed(self.gga(),self.stamp+.2)['covariance'])
        for message in (self.gst(''),self.gst('240000'),self.gst()[:-2]+'FF'):
            builder=ObservationBuilder()
            builder.feed(self.gga(),self.stamp)
            self.assertIsNone(builder.feed(message,self.stamp+.1))
        self.builder.feed(self.gga(0),self.stamp)
        update=self.builder.feed(self.gst(),self.stamp+.1)
        self.assertIsNotNone(update)
        self.assertFalse(update['valid_position'])
        self.assertIsNone(update['latitude'])
    def test_midnight_gst_must_match_exact_epoch(self):
        self.builder.feed(self.gst('235959.95'),self.stamp)
        self.assertIsNone(self.builder.feed(self.gga(time='000000.00'),self.stamp+.01)['covariance'])
        update=self.builder.feed(self.gst('000000.000'),self.stamp+.02)
        self.assertIsNotNone(update)
        self.assertAlmostEqual(update['horizontal_std_m'],.2)
    def test_unrepresentable_gst_covariance_before_gga_stays_unknown(self):
        for values in ('1e200,0.2,0.3','0.1,1e200,0.3','0.1,0.2,1e200',
                       '0.1,0.2,1e-200'):
            builder=ObservationBuilder()
            builder.feed(self.gst(values=values),self.stamp)
            result=builder.feed(self.gga(),self.stamp+.01)
            json.dumps(result,allow_nan=False)
            self.assertIsNone(result['covariance'])
            self.assertIsNone(result['horizontal_std_m'])
    def test_unrepresentable_gst_covariance_after_gga_allows_valid_retry(self):
        for values in ('1e200,0.2,0.3','0.1,1e200,0.3','0.1,0.2,1e200',
                       '0.1,0.2,1e-200'):
            builder=ObservationBuilder()
            original=builder.feed(self.gga(),self.stamp)
            update=builder.feed(self.gst(values=values),self.stamp+.01)
            json.dumps(update,allow_nan=False)
            self.assertIsNone(update)
            self.assertIsNone(original['covariance'])
            valid=builder.feed(self.gst(),self.stamp+.02)
            json.dumps(valid,allow_nan=False)
            self.assertAlmostEqual(valid['horizontal_std_m'],.2)
    def test_no_fix_does_not_carry_old_position(self):
        self.builder.feed(self.gga(),self.stamp)
        result=self.builder.feed(self.gga(0),self.stamp+.01)
        self.assertFalse(result['valid_position'])
        self.assertIsNone(result['latitude'])
    def test_date_unavailable_stale_rmc_and_bad_checksum(self):
        self.assertIsNone(ObservationBuilder().feed(self.gga(),self.stamp)['stamp'])
        self.assertIsNone(self.builder.feed(self.gga(),self.stamp+3)['stamp'])
        self.assertIsNone(self.builder.feed(self.gga()[:-2]+'FF',self.stamp))
    def test_midnight_rollover(self):
        now=datetime(2026,10,5,0,0,0,tzinfo=timezone.utc).timestamp()
        self.builder.feed(sentence('GNRMC,235959.90,A,3954.0000,N,11618.0000,E,0,0,041026,,,A'),now-.1)
        self.assertEqual(self.builder.feed(self.gga(time='000000.00'),now+.05)['stamp'],now)

if __name__=='__main__':unittest.main()
