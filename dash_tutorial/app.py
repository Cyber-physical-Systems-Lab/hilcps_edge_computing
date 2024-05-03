# Import packages
from time import sleep
from dash import Dash, html, dash_table, dcc, callback, Output, Input
import pandas as pd
import dash_bootstrap_components as dbc
import rclpy  # ROS 2 Python client library
from rclpy.node import Node
from std_msgs.msg import UInt8  # ROS 2 standard message type
import threading
from rclpy.action import ActionClient
from interfaces.action import MoveHand
from interfaces.msg import SpaceState
from std_msgs.msg import Bool


OFFLINE = "Offline"
STARTSPACE = "/spinningfactory/startspace_state"
WORKSPACE = "/spinningfactory/workspace_state"
HAND_SUFFIX = "_hand"

rhand_state = OFFLINE
startspace_state = OFFLINE
workspace_state = OFFLINE
startspace_hand = OFFLINE
workspace_hand = OFFLINE

rhand_lock = threading.Lock()
startspace_lock = threading.Lock()
workspace_lock = threading.Lock()
startspace_hand_lock = threading.Lock()
workspace_hand_lock = threading.Lock()

class MySubscriber(Node):
    

    def __init__(self):
        super().__init__('my_subscriber')
    
        self.mycobot_states = {
            0 : "IDLE",
            1 : "HANDLE STARTSPACE",
            2 : "HANDLE WORKSPACE",
            3 : "BACKING"
        }

        self.space_states = {
            0: "EMPTY",
            1: "BLUE PLACED",
            2: "RED PLACED",
            3: "BLUE AND RED PLACED",
            4: "ERROR"
        }
    
        self.hand_states = {
            False: "NO HUMAN",
            True: "HUMAN INTERACTION"
        }

        self.rhand_subscription = self.create_subscription(
            UInt8,
            '/spinningfactory/mycobotstate',
            self.robotstate_on_receive,
            10
        )

        self.startspace_subscription = self.create_subscription(
            SpaceState,
            STARTSPACE,
            self.startspace_on_receive,
            10
        )

        self.workspace_subscription = self.create_subscription(
            SpaceState,
            WORKSPACE,
            self.workspace_on_receive,
            10
        )

        self.startspace_hand_subscription = self.create_subscription(
            Bool,
            STARTSPACE + HAND_SUFFIX,
            self.startspace_hand_on_receive,
            10
        )

        self.workspace_hand_subscription = self.create_subscription(
            Bool,
            WORKSPACE + HAND_SUFFIX,
            self.workspace_hand_on_receive,
            10
        )

        # Timer to trigger timeout function
        self.timeout_threshold = 5.0
        self.rhand_timer = self.create_timer(self.timeout_threshold, self.rhand_timeout)
        self.startspace_timer = self.create_timer(self.timeout_threshold, self.startspace_timeout)
        self.workspace_timer = self.create_timer(self.timeout_threshold, self.workspace_timeout)

        # Actions
        self.startspace_client = ActionClient(self, MoveHand, 'handleStartSpace')
        self.workspace_client = ActionClient(self, MoveHand, 'handleWorkSpace')
        self.backoff_client = ActionClient(self, MoveHand, 'handleReturnToInit')

    def robotstate_on_receive(self, msg):
        self.rhand_timer.reset()
        global rhand_state
        with rhand_lock:
            print(f"{msg.data}")
            rhand_state = f"{self.mycobot_states[msg.data]} ({msg.data})"

    def startspace_on_receive(self, msg):
        self.startspace_timer.reset()
        global startspace_state
        with startspace_lock:
            startspace_state = f"{self.space_states[msg.state]} ({msg.state})"
    
    def workspace_on_receive(self, msg):
        self.workspace_timer.reset()
        global workspace_state
        with workspace_lock:
            workspace_state = f"{self.space_states[msg.state]} ({msg.state})"

    def startspace_hand_on_receive(self, msg):
        self.startspace_timer.reset()
        global startspace_hand
        with startspace_hand_lock:
            startspace_hand = f"{self.hand_states[msg.data]} ({msg.data})"


    def workspace_hand_on_receive(self, msg):
        self.workspace_timer.reset()
        global workspace_hand
        with workspace_hand_lock:
            workspace_hand = f"{self.hand_states[msg.data]} ({msg.data})"

    def rhand_timeout(self):
        self.get_logger().warn(f'RHAND: No message received for {self.timeout_threshold} seconds!')
        global rhand_state
        with rhand_lock:
            rhand_state = OFFLINE

    def startspace_timeout(self):
        self.get_logger().warn(f'STARTSPACE: No message received for {self.timeout_threshold} seconds!')
        global startspace_state
        global startspace_hand
        with startspace_lock:
            startspace_state = OFFLINE
            startspace_hand = OFFLINE

    def workspace_timeout(self):
        self.get_logger().warn(f'WORKSPACE: No message received for {self.timeout_threshold} seconds!')
        global workspace_state
        global workspace_hand
        with workspace_lock:
            workspace_state = OFFLINE
            workspace_hand = OFFLINE

    def trigger_startspace_action(self):
        self.startspace_client.wait_for_server()
        goal_msg = MoveHand.Goal()
        # Send goal to handleWorkSpace action server
        future = self.startspace_client.send_goal_async(goal_msg)
    
    def trigger_workspace_action(self):
        self.workspace_client.wait_for_server()
        goal_msg = MoveHand.Goal()
        # Send goal to handleWorkSpace action server
        future = self.workspace_client.send_goal_async(goal_msg)
    
    def trigger_backoff_action(self):
        self.backoff_client.wait_for_server()
        goal_msg = MoveHand.Goal()
        # Send goal to handleWorkSpace action server
        future = self.backoff_client.send_goal_async(goal_msg)
        

class ROS2ListenerThread(threading.Thread):
    def __init__(self):
        super().__init__(name="ros2_listener")
        self.subscriber = MySubscriber()
        self._stop_event = rclpy.Future()

    def stop(self):
        self._stop_event.set_result("")

    def run(self):
        while not self._stop_event.done():
            rclpy.spin_until_future_complete(self.subscriber,self._stop_event)
        self.subscriber.destroy_node()
        rclpy.shutdown()
    
    def get_node(self):
        return self.subscriber
        


def setup_dashboard(node: MySubscriber):
    
    
    # Incorporate data
    df = pd.read_csv('https://raw.githubusercontent.com/plotly/datasets/master/gapminder2007.csv')

    external_stylesheets = [dbc.themes.CERULEAN]
    app = Dash(__name__, external_stylesheets=external_stylesheets)

    # App layout
    app.layout = dbc.Container([
        dcc.Interval(id='interval-component', interval=500),
        dbc.Row([
            html.Div('Spinning factory analytics', className="text-primary text-center fs-3")
        ]),

        # Graph and state indicators
        dbc.Row([
        dbc.Col(dash_table.DataTable(
            data=df.to_dict('records'),
            page_size=12,
            style_table={'overflowX': 'auto'}
        )),  # 1/4 width of the page

        dbc.Col([
            # Robotic hand state
            html.Div([
                html.H3("Robotic hand state", style={'color': 'black'}),
                html.Div(id='update_rhand', children=rhand_state, style={'backgroundColor': '#f2f2f2'})
            ], style={'marginBottom': '20px'}),  # Add margin bottom

            # StartSpace state
            html.Div([
                html.H3("StartSpace"),
                html.Div(id='update_startspace', children=startspace_state, style={'backgroundColor': '#f2f2f2'})
            ], style={'marginBottom': '10px'}),  # Add margin bottom

            # StartSpace hand
            html.Div([
                html.H3("StartSpace hand"),
                html.Div(id='update_startspace_hand', children=startspace_state, style={'backgroundColor': '#f2f2f2'})
            ], style={'marginBottom': '20px'}),  # Add margin bottom

            # WorkSpace state
            html.Div([
                html.H3("WorkSpace", style={'color': 'darkred'}),
                html.Div(id='update_workspace', children=workspace_state, style={'backgroundColor': '#f2f2f2'})
            ], style={'marginBottom': '10px'}),  # Add margin bottom

            html.Div([
                html.H3("WorkSpace hand", style={'color': 'darkred'}),
                html.Div(id='update_workspace_hand', children=workspace_state, style={'backgroundColor': '#f2f2f2'})
            ], style={'marginBottom': '20px'})  # Add margin bottom
        ], width=3)  # 3/4 width of the page
    ]),

        # DF and buttons
        dbc.Row([
            dbc.Col([dash_table.DataTable(data=df.to_dict('records'), page_size=12, style_table={'overflowX': 'auto'})]),
            dbc.Col([
                    html.Button("Move from Startspace", id="btn-startspace", className="mr-2 mb-2 d-block"),
                    html.Button("Move from Workspace", id="btn-workspace", className="mr-2 mb-2 d-block"),
                    html.Button("Back Off", id="btn-backoff", className="mr-2 mb-2 d-block"),
                    dbc.Row([
                        dbc.Col([
                            html.Button("Clear", id="btn-clear", className="mr-2 mb-2 d-block")
                        ]),
                        dbc.Col([
                            html.Button("Save", id="btn-save", className="mr-2 mb-2 d-block")
                        ])
                    ], className="mt-5")
            ], width=3, style={'marginTop': 'auto', 'marginBottom': '25px'}),


        ]),

        # for no-output callbacks
        html.Div(id="hidden-div1", style={"display":"none"}),
        html.Div(id="hidden-div2", style={"display":"none"}),
        html.Div(id="hidden-div3", style={"display":"none"}),


    ], fluid=True)
    @app.callback(
        Output('update_rhand', 'children'),
        [Input('interval-component', 'n_intervals')]  # This input is just a dummy input to trigger the callback periodically
    )
    def update_rhand_state(n_intervals):
        return rhand_state
    
    @app.callback(
        Output('update_startspace', 'children'),
        [Input('interval-component', 'n_intervals')] 
    )
    def update_startspace_state(n_intervals):
        return startspace_state
    
    @app.callback(
        Output('update_workspace', 'children'),
        [Input('interval-component', 'n_intervals')] 
    )
    def update_workspace_state(n_intervals):
        return workspace_state
    
    @app.callback(
        Output('update_startspace_hand', 'children'),
        [Input('interval-component', 'n_intervals')] 
    )
    def update_startspace_hand(n_intervals):
        return startspace_hand
    
    @app.callback(
        Output('update_workspace_hand', 'children'),
        [Input('interval-component', 'n_intervals')] 
    )

    def update_workspace_hand(n_intervals):
        return workspace_hand
    
    # Callback functions for Dash buttons
    @app.callback(
        Output('hidden-div1', 'children'),  # Add a dummy output to trigger the callback
        [Input("btn-startspace", 'n_clicks')]
    )
    def start_space_callback(n_clicks):
        node.trigger_startspace_action()
        return ""

    @app.callback(
        Output('hidden-div2', 'children'),
        [Input('btn-workspace', 'n_clicks')]
    )
    def workspace_callback(n_clicks):
        node.trigger_workspace_action()
        return ""

    @app.callback(
        Output('hidden-div3', 'children'),
        [Input('btn-backoff', 'n_clicks')]
    )
    def backoff_callback(n_clicks):
        node.trigger_backoff_action()
        return ""
        
    return app


        
        

# Run the app
if __name__ == '__main__':
    rclpy.init()
    listener_thread = ROS2ListenerThread()
    app = setup_dashboard(listener_thread.get_node())
    try:
        listener_thread.start()
        sleep(2)
        app.run(debug=False)
    except KeyboardInterrupt:
        pass
    listener_thread.stop()