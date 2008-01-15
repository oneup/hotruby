$n = $native
$n.import "Box2D.Dynamics.*"
$n.import "Box2D.Collision.*"
$n.import "Box2D.Collision.Shapes.*"
$n.import "Box2D.Dynamics.Joints.*"
$n.import "Box2D.Dynamics.Contacts.*"
$n.import "Box2D.Common.Math.*"
$n.import "flash.events.*"
$n.import "flash.display.*"
$n.import "flash.text.*"
$n.import "General.*"
$n.import "TestBed.*"

class Box2D
	def initialize
		@curr_id = 1
		@curr_test = nil
		
		add_fps_counter
		add_sprite
		add_instructions_text
		add_about_text
		add_input_fix_sprite
		add_listener
	end

	def add_listener
		update = Proc.new{|evt|
			# clear for rendering
			@sprite.graphics.clear
			
			# toggle between tests
			if $n.Input.isKeyPressed 39 then # Right Arrow
				@curr_id = @curr_id + 1
				@curr_test = nil
			elsif $n.Input.isKeyPressed 37 then # Left Arrow
				@curr_id = @curr_id - 1
				@curr_test = nil
			# Reset
			elsif $n.Input.isKeyPressed 82 then # R
				@curr_test = nil
			end
			
			# if nil, set new test
			if nil == @curr_test then
				case @curr_id
				# Bridge
				when 0
					@curr_test = $n.TestBridge.new
				# Example
				when 1
					@curr_test = $n.TestExample.new
				# Ragdoll
				when 2
					@curr_test = $n.TestRagdoll.new
				# Compound
				when 3
					@curr_test = $n.TestCompound.new
				# Stack
				when 4
					@curr_test = $n.TestStack.new
				# Crank
				when 5
					@curr_test = $n.TestCrank.new
				# Pulley
				when 6
					@curr_test = $n.TestPulley.new
				# Gears
				when 7
					@curr_test = $n.TestGears.new
				# Wrap around
				else
					if @curr_id < 0 then
						@curr_id = 7
						@curr_test = $n.TestGears.new
					else
						@curr_id = 0
						@curr_test = $n.TestBridge.new
					end
				end
			end
			
			# update current test
			@curr_test.Update
			
			# Update input (last)
			$n.Input.update
			
			# update counter and limit framerate
			@fps_counter.update
			$n.FRateLimiter.limitFrame 30
		}

		$n._root.addEventListener $n.Event.ENTER_FRAME, update, false, 0, true
	end

	def add_fps_counter
		@fps_counter = $n.FpsCounter.new
		@fps_counter.x = 7
		@fps_counter.y = 5
		$n.Main.m_fpsCounter = @fps_counter
		$n._root.addChildAt @fps_counter, 0
	end
	
	def add_sprite
		@sprite = $n.Sprite.new
		$n.Main.m_sprite = @sprite
		$n._root.addChild @sprite

		@input = $n.Input.new @sprite
	end

	def add_instructions_text
		#Instructions Text
		instructions_text = $n.TextField.new

		instructions_text_format = $n.TextFormat.new "Arial", 16, 0xffffff, false, false, false
		instructions_text_format.align = $n.TextFormatAlign.RIGHT

		instructions_text.defaultTextFormat = instructions_text_format
		instructions_text.x = 140
		instructions_text.y = 4.5
		instructions_text.width = 495
		instructions_text.height = 61
		instructions_text.text = "Box2DFlashAS3 examples: \n'Left'/'Right' arrows to go to previous/next example. \n'R' to reset."
		$n._root.addChild instructions_text
	end

	# textfield pointer
	def add_about_text
		aboutTextFormat = $n.TextFormat.new "Arial", 16, 0x00CCFF, true, false, false
		aboutTextFormat.align = $n.TextFormatAlign.RIGHT

		about_text = $n.TextField.new
		about_text.defaultTextFormat = aboutTextFormat
		about_text.x = 434
		about_text.y = 71
		about_text.width = 200
		about_text.height = 30
		$n.Main.m_aboutText = about_text
		$n._root.addChild about_text
	end

	def add_input_fix_sprite
		# Make a big invisible box to cover the stage so that input focus doesn't change when mousing over the textfields
		# (Please let me know if there's a better way to solve this problem) (:
		inputFixSprite = $n.Sprite.new
		inputFixSprite.graphics.lineStyle 0,0,0
		inputFixSprite.graphics.beginFill 0,0
		inputFixSprite.graphics.moveTo -10000, -10000
		inputFixSprite.graphics.lineTo 10000, -10000
		inputFixSprite.graphics.lineTo 10000, 10000
		inputFixSprite.graphics.lineTo -10000, 10000
		inputFixSprite.graphics.endFill
		$n._root.addChild inputFixSprite
	end
end

Box2D.new
