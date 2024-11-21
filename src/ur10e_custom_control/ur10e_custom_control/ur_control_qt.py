import sys
import inspect
import ast
from PyQt5.QtWidgets import *
from typing import Optional, Callable, Any
from ur_robot_node import URRobot
from ur10e_typedefs import URService
from functools import partial

class URControlQtWindow(QMainWindow): # TODO: make ROS node
    def __init__(self):
        super().__init__()
        
        self._robot: Optional[URRobot] = None

        # Set the main window properties
        self.setWindowTitle("UR Control Node")
        self.setGeometry(0, 0, 400, 400)

        self.tab_widget = QTabWidget()
        self.setCentralWidget(self.tab_widget)

        self._launch_buttons = []

        self.robot_tab = self._create_tab(name = "Robot Tab", tab_create_func = self._conf_robot_tab)
        self.service_tab = self._create_tab(name = "Service Tab", tab_create_func = self._conf_service_tab)
    
    def _create_tab(self, name: str, tab_create_func: Optional[Callable[[QLayout], None]] = None):
        _tab_widget = QWidget()
        _tab_layout = QVBoxLayout()
        _tab_widget.setLayout(_tab_layout)

        if tab_create_func is not None:
           tab_create_func(_tab_layout)

        self.tab_widget.addTab(_tab_widget, name)

        return _tab_widget
    
    def _conf_robot_tab(self, layout: QLayout) -> None:
        def __init_robot(button: QPushButton):
            self._robot = URRobot("ur_node")
            button.setEnabled(False) # disable button so we can't re-initialize the class
            for button in self._launch_buttons:
                # Now that robot node is active, we can enable services
                # NOTE: this isn't actually a pre-requisite, but we use
                # the robot class' methods for processing of services, and
                # the robot node being able to run only can occur when UR
                # control has been activated...
                button.setEnabled(True)
            
        def __send_trajectory(button: QPushButton):
            trajectory_kwargs = self._create_signature_kwargs_from_dialog(
                "send_trajectory args", inspect.signature(self._robot.send_trajectory)
            )
            if trajectory_kwargs is not None:
                print(f"Trajectory: {trajectory_kwargs}")
                self._robot.send_trajectory(**trajectory_kwargs)

        # Launch tab
        _launch_map: dict[QPushButton, Callable] = {
            QPushButton("INIT ROBOT", self): __init_robot,
            QPushButton("SEND_TRAJECTORY", self): __send_trajectory
        }

        for button, callback_func in _launch_map.items():
            button.clicked.connect(partial(callback_func, button))
            layout.addWidget(button)

    def _conf_service_tab(self, layout: QLayout) -> None:
        def __service_name(_service: URService): return str(_service).split("SRV_")[-1]
        for service in URService.URServices:
            _button = QPushButton(__service_name(service), self)
            _button.clicked.connect(partial(self._request_service, service))
            _button.setEnabled(False) # disable until launch is initiated
            self._launch_buttons.append(_button)
            layout.addWidget(_button)

    def _request_service(self, service: URService.URServiceType): 
        _service = URService._UR_SERVICE_MAP.get(service)
        _fields_and_field_types = _service.Request.get_fields_and_field_types()
        if _fields_and_field_types == {}:
            print(_service.Request())
            print(f"Request return: {self._robot.call_service(service, _service.Request())}")
        else:
            service_kwargs = self._create_service_kwargs_from_dialog(service, _fields_and_field_types)
            if service_kwargs is not None:
                print(_service.Request(**service_kwargs))
                print(f"Request return: {self._robot.call_service(service, _service.Request(**service_kwargs))}")

    def _create_signature_kwargs_from_dialog(self, name: str, signature: inspect.Signature):
        if signature.parameters == {}:
            return {}
        else:
            # Create dialog window
            _dialog = QDialog()

            # Title it with the name of the service
            _dialog.setWindowTitle(name + " arguments")
            
            # Create grid, where column 0 has the input fields 
            # and column 1 has the user inputs
            _layout = QGridLayout()
            _dialog.setLayout(_layout)

            # Store user inputs, which will then be mapped to 
            # typed inputs
            _user_inputs: dict[str, QLineEdit] = {}
            _typed_inputs: dict[str, Any] = {}

            for cnt, (field, field_type) in enumerate(signature.parameters.items()):
                _user_inputs[field] = QLineEdit()
                _layout.addWidget(QLabel(field + f" [type: {field_type.annotation}]"), cnt, 0)
                _layout.addWidget(_user_inputs[field], cnt, 1)
            
            _apply_button = QPushButton("Apply", _dialog)
            _apply_button.clicked.connect(_dialog.accept)

            _cancel_button = QPushButton("Cancel", _dialog)
            _cancel_button.clicked.connect(_dialog.reject)

            _layout.addWidget(_apply_button, cnt + 1, 0)
            _layout.addWidget(_cancel_button, cnt + 1, 1)

            # Execute dialog window
            if _dialog.exec_():
                # Custom inputs requested to be applied
                for field, field_type in signature.parameters.items():
                    _user_input = _user_inputs[field].text()
                    if _user_input != '':
                        import types
                        if isinstance(field_type.annotation, types.GenericAlias):
                            _typed_inputs[field] = self._typed_data(_user_input, str(field_type.annotation))
                        else:
                            _typed_inputs[field] = field_type.annotation(_user_inputs)
                
                return _typed_inputs
            else:
                # Service request canceled
                return None

    def _create_service_kwargs_from_dialog(self, service: URService.URServiceType, fields_and_field_types: dict[str, str]) -> dict[str, Any]:
        # Create dialog window
        _dialog = QDialog()

        # Title it with the name of the service
        _dialog.setWindowTitle(service.value + " arguments")
        
        # Create grid, where column 0 has the input fields 
        # and column 1 has the user inputs
        _layout = QGridLayout()
        _dialog.setLayout(_layout)

        # Store user inputs, which will then be mapped to 
        # typed inputs
        _user_inputs: dict[str, QLineEdit] = {}
        _typed_inputs: dict[str, Any] = {}

        for cnt, (field, field_type) in enumerate(fields_and_field_types.items()):
            _user_inputs[field] = QLineEdit()
            _layout.addWidget(QLabel(field + f" [type: {field_type}]"), cnt, 0)
            _layout.addWidget(_user_inputs[field], cnt, 1)
        
        _apply_button = QPushButton("Apply", _dialog)
        _apply_button.clicked.connect(_dialog.accept)

        _cancel_button = QPushButton("Cancel", _dialog)
        _cancel_button.clicked.connect(_dialog.reject)

        _layout.addWidget(_apply_button, cnt + 1, 0)
        _layout.addWidget(_cancel_button, cnt + 1, 1)

        # Execute dialog window
        if _dialog.exec_():
            # Custom inputs requested to be applied
            for field, field_type in fields_and_field_types.items():
                _user_input = _user_inputs[field].text()
                if _user_input != '':
                    _typed_inputs[field] = self._typed_data(_user_input, field_type)

            return _typed_inputs
        else:
            # Service request canceled
            return None
    
    def _typed_data(self, text: str, field_type: str):
        _INT_TYPE = ('int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64')
        _FLOAT_TYPE = ('float', 'double')

        if field_type == 'string':
            return text
        elif "sequence" in field_type:
            _inner_type = field_type.split("<")[-1].split(">")[0] # assuming format of `sequence<type>`...
            return [self._typed_data(x.strip(), field_type = _inner_type) for x in text.split(",")]
        elif "list" == field_type[:4].lower():
            _inner_type = field_type[4:][1:-1]
            if "list" in _inner_type:
                return [self._typed_data(text[1:-1].strip(), field_type = _inner_type)]
            else:
                return [self._typed_data(x.strip(), field_type = _inner_type) for x in text[1:-1].split(",")]
        elif "boolean" in field_type:
            return True if text.lower() == "true" else False
        elif field_type in _INT_TYPE:
            return int(text)
        elif field_type in _FLOAT_TYPE:
            return float(text)
        elif "Duration" in field_type:
            from builtin_interfaces.msg import Duration
            return Duration(
                sec = int(float(text)), 
                nanosec = int((float(text) - int(float(text))) * 1e9)
            )
        else:
            raise TypeError(f"Unknown type: {field_type}")

def main():
    # Create the application
    app = QApplication(sys.argv)
    
    # Create the main window
    main_window = URControlQtWindow()
    main_window.show()
    
    # Run the application's event loop
    sys.exit(app.exec_())