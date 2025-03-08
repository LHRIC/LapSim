import unittest #Step 1: import unnitest, you do not need to install anything
from unittest.mock import patch, MagicMock
import numpy as np
from kinematics.kinematic_model import KinematicModel

#you need to initialize unnitTest class
class TestKinematicModel(unittest.TestCase):
    def setUp(self):
        self.model = KinematicModel()
        self.mock_hardpoints = {
            'unit': 'mm',
            'steering_rack_delta': [39.5, 51],
            'front_shock_travel': [-20, 20, 51],
            'rear_shock_travel': [-20, 20, 51],
            'front_arb_exists': True,
            'rear_arb_exists': False,
            'front_left': {
                'FLLO': [9.467, 571.194, 129.857],  
                'FLLIF': [105.000, 225.100, 127.000],  
                'FLLIA': [-97.935, 244.639, 137.349],  
                'FLUO': [6.448, 555.481, 301.938],  
                'FLUIF': [74.885, 271.488, 294.354],  
                'FLUIA': [-116.813, 274.845, 261.826], 
                'FLOT': [70.600, 582.000, 171.965],  
                'FLIT': [79.500, 220.980, 166.476],  
                'FLCP': [0.000, 610.000, 0.000],  
                'FLWC': [0.000, 610.000, 203.200],  
                'FLOP': [8.130, 539.539, 143.975],  
                'FLIP': [-12.465, 335.491, 385.469],  
                'FLBC': [-10.752, 219.514, 365.611],  
                'FLSO': [-15.059, 341.494, 415.830],  
                'FLSI': [-26.500, 222.500, 550.000],  
                'FLABC': [-10.730, 279.879, 265.247],  
                'FLALV': [-2.209, 279.879, 153.829], 
                'FLARB': [-47.046, 279.879, 150.000]  
            },
            'rear_left': {
                'RLLO': [-1535.000, 597.620, 114.945],
                'RLLIF': [-1250.620, 211.000, 105.000],
                'RLLIA': [-1478.000, 211.000, 105.000],
                'RLUO': [-1535.000, 578.736, 290.399],
                'RLUIF': [-1240.000, 287.300, 230.200],
                'RLUIA': [-1495.000, 287.006, 230.200],
                'RLOT': [-1635.000, 595.000, 187.000],
                'RLIT': [-1492.734, 267.312, 154.625],
                'RLCP': [-1535.000, 610.000, 0.000],
                'RLWC': [-1535.000, 610.000, 203.200],
                'RLOP': [-1522.478, 556.566, 270.748],
                'RLIP': [-1374.439, 246.131, 142.550],
                'RLBC': [-1348.845, 200.416, 149.864],
                'RLSO': [-1397.002, 303.032, 197.613],
                'RLSI': [-1375.176, 310.000, 374.132],
                'RLABC': [-1380.000, 280.000, 250.000],
                'RLALV': [-1375.000, 280.000, 160.000],
                'RLARB': [-1370.000, 280.000, 150.000]
            }
        }
    # Step 2: Write a test function 
    # The patch() decorator / context manager makes it easy to mock classes or objects in a module under test. 
    # The object you specify will be replaced with a mock (or other object) during the test and restored when the test ends
    @patch('kinematics.kinematic_model.read_yaml')
    def test_from_hardpoints(self, mock_read_yaml):
        mock_read_yaml.return_value = self.mock_hardpoints
        self.model.from_hardpoints('any_filename')
        self.assertEqual(self.model.steering_rack_delta, [39.5, 51])
        self.assertEqual(self.model.front_shock_travel, [-20, 20, 51])
        self.assertEqual(self.model.rear_shock_travel, [-20, 20, 51])
        self.assertTrue(self.model.front_arb_exists)
        self.assertFalse(self.model.rear_arb_exists)
        self.assertIsNotNone(self.model.front_left)
        self.assertIsNotNone(self.model.rear_left)
        self.assertEqual(self.model.fl_hardpoints['LO'], [9.467, 571.194, 129.857])
        self.assertEqual(self.model.rl_hardpoints['LO'], [-1535.000, 597.620, 114.945])
        self.assertIsNotNone(self.model.front)
        self.assertIsNotNone(self.model.rear)

    @patch('kinematics.kinematic_model.kinematic_solver')
    def test_generate_model(self, mock_solver):
        mock_corner = MagicMock()
        mock_corner.shock_inboard.pos = np.array([0, 0, 0])
        mock_corner.shock_outboard.pos = np.array([0, 0, 3])  
        mock_corner.contact_patch.pos = np.array([1, 2, 0])
        mock_corner.inboard_tie.pos = np.array([0, 0, 0])
        mock_corner.inboard_tie.initial_pos = np.array([0, 0, 0])
        mock_corner.wheel_sys.delta_angle.return_value = np.array([0, 0, 0])
        mock_corner.shock.length.return_value = 3.0
        mock_corner.dependent_objects = []
        mock_corner.residual_objects = []
        mock_corner.update_objects = []
        mock_linear = MagicMock() #used for classes with mock() properties 
        mock_corner.linear = mock_linear
        steering_rack_delta = [10, 3]  # 10mm of travel, 3 points
        shock_travel = [-5, 5, 3]     # -5 to 5mm travel, 3 points
        result = self.model._generate_model(mock_corner, steering_rack_delta, shock_travel)
        self.assertEqual(result.shape, (3, 3, 11))
        self.assertEqual(mock_solver.call_count, 9)
        self.assertTrue(mock_linear.length is not None)
        self.assertEqual(mock_corner.inboard_tie.translate.call_count, 9)

    @patch('kinematics.kinematic_model.RegularGridInterpolator')
    def test_interpolate(self, mock_interp_class):
        surrogate_array = np.zeros((3, 3, 11))
        surrogate_array[:, 0, 0] = np.array([-5, 0, 5])
        surrogate_array[0, :, 1] = np.array([-10, 0, 10])
        mock_interp = MagicMock()
        mock_interp_class.return_value = mock_interp
        mock_interp.return_value = np.array([[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]])
        result = self.model.interpolate(2.5, 5.0, surrogate_array)
        mock_interp_class.assert_called_once()
        np.testing.assert_array_equal(mock_interp_class.call_args[0][0][0], np.array([-5, 0, 5]))
        np.testing.assert_array_equal(mock_interp_class.call_args[0][0][1], np.array([-10, 0, 10]))
        np.testing.assert_array_equal(mock_interp_class.call_args[0][1], surrogate_array)
        np.testing.assert_array_equal(mock_interp.call_args[0][0], np.array([2.5, 5.0]))
        np.testing.assert_array_equal(result, np.array([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]))

    @patch('kinematics.kinematic_model.read_yaml')
    def test_front_rear_models_different(self, mock_read_yaml):
        mock_read_yaml.return_value = self.mock_hardpoints
        with patch.object(KinematicModel, '_generate_model') as mock_gen:
            mock_gen.side_effect = [
                np.zeros((51, 51, 11)),
                np.zeros((51, 1, 11))
            ]
            
            self.model.from_hardpoints('any_filename')
            self.assertEqual(mock_gen.call_count, 2)
            front_call_args = mock_gen.call_args_list[0][0]
            self.assertEqual(front_call_args[1], [39.5, 51])  
            self.assertEqual(front_call_args[2], [-20, 20, 51]) 
            rear_call_args = mock_gen.call_args_list[1][0]
            self.assertIsNone(rear_call_args[1])  # No steering for rear
            self.assertEqual(rear_call_args[2], [-20, 20, 51])  # rear_shock_travel

    @patch('kinematics.kinematic_model.read_yaml')
    def test_shock_length_calculation(self, mock_read_yaml):
        mock_read_yaml.return_value = self.mock_hardpoints
        with patch('kinematics.kinematic_model.SuspensionCorner') as mock_corner_class:
            mock_fl = MagicMock()
            mock_rl = MagicMock()
            mock_corner_class.side_effect = [mock_fl, mock_rl]
            with patch.object(KinematicModel, '_generate_model') as mock_gen:
                mock_gen.side_effect = [
                    np.zeros((51, 51, 11)),  # Front model
                    np.zeros((51, 1, 11))    # Rear model
                ]
                
                self.model.from_hardpoints('any_filename')
                self.assertEqual(mock_gen.call_count, 2)
                front_call_args = mock_gen.call_args_list[0][0]
                self.assertEqual(front_call_args[0], mock_fl)
                self.assertEqual(front_call_args[1], [39.5, 51])
                self.assertEqual(front_call_args[2], [-20, 20, 51])
                rear_call_args = mock_gen.call_args_list[1][0]
                self.assertEqual(rear_call_args[0], mock_rl)
                self.assertIsNone(rear_call_args[1])
                self.assertEqual(rear_call_args[2], [-20, 20, 51])

    def test_snip_function(self):
        corner_dict = {
            'FLLO': [1, 2, 3],
            'FLHI': [4, 5, 6],
            'FLTEST': [7, 8, 9]
        }
        def test_snip(corner_dict):
            hardpoints_snipped = {}
            for key, value in corner_dict.items():
                new_key = key[2:]  
                hardpoints_snipped[new_key] = value
            return hardpoints_snipped
        
        result = test_snip(corner_dict)
        
        # Check the result
        expected = {
            'LO': [1, 2, 3],
            'HI': [4, 5, 6],
            'TEST': [7, 8, 9]
        }
        self.assertEqual(result, expected)

    @patch('kinematics.kinematic_model.RegularGridInterpolator')
    def test_interpolate_edge_cases(self, mock_interp_class):
        surrogate_array = np.zeros((3, 3, 11))
        surrogate_array[:, 0, 0] = np.array([-5, 0, 5])  # shock values
        surrogate_array[0, :, 1] = np.array([-10, 0, 10])  # steer values
        mock_interp = MagicMock()
        mock_interp_class.return_value = mock_interp
        mock_interp.return_value = np.array([[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]])
        result1 = self.model.interpolate(0, 0, surrogate_array)
        np.testing.assert_array_equal(result1, np.array([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]))
        np.testing.assert_array_equal(mock_interp.call_args[0][0], np.array([0, 0]))
        mock_interp.return_value = np.array([[2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]])
        result2 = self.model.interpolate(5, 10, surrogate_array)
        np.testing.assert_array_equal(result2, np.array([2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]))
        np.testing.assert_array_equal(mock_interp.call_args[0][0], np.array([5, 10]))
        mock_interp.return_value = np.array([[np.nan, np.nan, np.nan, np.nan, np.nan, 
                                             np.nan, np.nan, np.nan, np.nan, np.nan, np.nan]])
        result3 = self.model.interpolate(10, 20, surrogate_array)
        self.assertTrue(np.isnan(result3).all())
        np.testing.assert_array_equal(mock_interp.call_args[0][0], np.array([10, 20]))

    @patch('kinematics.kinematic_model.read_yaml')
    def test_model_dimensions(self, mock_read_yaml):
        mock_read_yaml.return_value = self.mock_hardpoints
        
        # Mock SuspensionCorner to avoid actual initialization
        with patch('kinematics.kinematic_model.SuspensionCorner') as mock_corner_class:
            mock_fl = MagicMock()
            mock_rl = MagicMock()
            mock_corner_class.side_effect = [mock_fl, mock_rl]
            with patch.object(KinematicModel, '_generate_model') as mock_gen:
                mock_gen.side_effect = [
                    np.zeros((51, 51, 11)),  # Front model: matches front_shock_travel[2] and steering_rack_delta[1]
                    np.zeros((51, 1, 11))    # Rear model: matches rear_shock_travel[2] and 1 (no steering)
                ]
                
                self.model.from_hardpoints('any_filename')
                self.assertEqual(self.model.front.shape, (51, 51, 11))
                self.assertEqual(self.model.rear.shape, (51, 1, 11))
                self.assertEqual(mock_gen.call_count, 2)
                front_call = mock_gen.call_args_list[0]
                self.assertEqual(front_call[0][1], [39.5, 51])
                self.assertEqual(front_call[0][2], [-20, 20, 51])
                rear_call = mock_gen.call_args_list[1]
                self.assertIsNone(rear_call[0][1])
                self.assertEqual(rear_call[0][2], [-20, 20, 51])

if __name__ == '__main__':
    unittest.main()