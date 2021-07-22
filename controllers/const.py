class parameter:

	def __init__(self):

		def return_total_step(self):
			total_step = (self.duration * 60 * 1000)/self.TIME_STEP
			return total_step

		self.V = 12.0

		# ROBOT CONTROL PARAMETER
		self.distance_Kp = 5.0 # 5.0
		self.distance_Kd = 0.1 # 0.1
		self.degree_Kp = 0.2
		self.degree_Kd = 80
		self.MAX_THRUST = 20.0
		self.TIME_STEP = 16

		# WAY POINT
		self.way_point_file = './way_point/hirako_target.csv'

		# MODE
		self.data_log_mode = True
		self.debug_mode = False
		self.state_display_mode = False
		self.random_disturbance_mode = False
		self.gps_error_mode = False

		# CONTROL MODE
		self.control_mode = 0
		# 0:OMNICONTROL MODE, 1:FIXED HEAD CONTROL MODE, 2: DIAGNALCONTROL MODE, 3: OCT-DIRECTIONAL

		# CONTROL STRATEGY
		self.policy = 1
		# 0:SIMPLE STRATEGY, 1:FLEX STRATEGY

		# OTHER
		self.main_target_distance_torelance = 3.0
		self.temp_target_distance_torelance = 1.5
		self.head_torelance = 5.0
		self.duration = 1 # [min]
		self.total_step = return_total_step(self)
		self.workspace = 'C:/Users/is0232xf/OneDrive - 学校法人立命館/デスクトップ/exp/'
