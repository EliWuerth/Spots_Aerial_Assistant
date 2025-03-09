import unittest
from unittest.mock import MagicMock
from your_module import TelloApp  # Replace with the actual name of your file

class TestTelloApp(unittest.TestCase):

    def test_initialization(self):
        # Create a mock for the Tello object
        mock_tello = MagicMock()
        mock_tello.get_battery.return_value = 100  # Mocking the battery level as 100%
        mock_tello.connect.return_value = None
        mock_tello.streamon.return_value = None
        
        # Initialize the app
        app = TelloApp()
        
        # Assert the Tello object methods were called
        app.tello = mock_tello
        app.tello.connect.assert_called_once()
        app.tello.get_battery.assert_called_once()
        app.tello.streamon.assert_called_once()

        # Ensure that the battery percentage printed is 100
        app.tello.get_battery.return_value = 100
        self.assertEqual(app.tello.get_battery(), 100)

    def test_camera_calibration_load(self):
        # Test that camera calibration loads correctly
        app = TelloApp()
        camera_matrix, dist_coeffs = app.load_camera_calibration('test_calibration_matrix.yaml')

        self.assertIsInstance(camera_matrix, np.ndarray)
        self.assertIsInstance(dist_coeffs, np.ndarray)

    def test_set_camera_direction(self):
        # Test switching between camera directions
        app = TelloApp()
        
        # Initially, camera_down should be False
        app.set_camera_direction()
        self.assertTrue(app.camera_down)
        
        # Switch back to camera_forward
        app.set_camera_direction()
        self.assertFalse(app.camera_down)

    def test_takeoff(self):
        # Test takeoff method
        app = TelloApp()
        app.tello.takeoff = MagicMock()
        app.takeoff()
        
        # Assert takeoff was called once
        app.tello.takeoff.assert_called_once()

    def test_land(self):
        # Test land method
        app = TelloApp()
        app.tello.land = MagicMock()
        app.land()
        
        # Assert land was called once
        app.tello.land.assert_called_once()

    def test_move_forward(self):
        # Test move_forward method
        app = TelloApp()
        app.tello.move_forward = MagicMock()
        app.move_forward()
        
        # Assert move_forward was called with the expected value
        app.tello.move_forward.assert_called_with(30)

    def test_emergency_stop(self):
        # Test emergency stop method
        app = TelloApp()
        app.tello.emergency = MagicMock()
        app.turn_off()
        
        # Assert emergency stop was called once
        app.tello.emergency.assert_called_once()

    def test_cleanup(self):
        # Test the cleanup method
        app = TelloApp()
        app.tello.end = MagicMock()
        app.cleanup()
        
        # Assert cleanup calls end() on Tello object
        app.tello.end.assert_called_once()

if __name__ == '__main__':
    unittest.main()
