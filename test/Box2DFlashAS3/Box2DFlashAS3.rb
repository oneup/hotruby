n = $native

n.import("Box2D.Dynamics.*")
n.import("Box2D.Collision.*")
n.import("Box2D.Collision.Shapes.*")
n.import("Box2D.Dynamics.Joints.*")
n.import("Box2D.Dynamics.Contacts.*")
n.import("Box2D.Common.Math.*")
n.import("flash.events.*")
n.import("flash.display.*")
n.import("flash.text.*")
n.import("General.*")
n.import("TestBed.*")

n.Main.m_fpsCounter = n.FpsCounter.new
m_currId = 0
m_currTest = nil
n.Main.m_sprite = nil
n.Main.m_aboutText = nil
# input
m_input = nil

update = Proc.new{|evt|
	# clear for rendering
	n.Main.m_sprite.graphics.clear()
	
	# toggle between tests
	if n.Input.isKeyPressed(39) then # Right Arrow
		m_currId = m_currId + 1
		m_currTest = nil
	elsif n.Input.isKeyPressed(37) then # Left Arrow
		m_currId = m_currId - 1
		m_currTest = nil
	# Reset
	elsif n.Input.isKeyPressed(82) then # R
		m_currTest = nil
	end
	
	# if nil, set new test
	if nil == m_currTest then
		case m_currId
		# Bridge
		when 0
			m_currTest = n.TestBridge.new
		# Example
		when 1
			m_currTest = n.TestExample.new
		# Ragdoll
		when 2
			m_currTest = n.TestRagdoll.new
		# Compound
		when 3
			m_currTest = n.TestCompound.new
		# Stack
		when 4
			m_currTest = n.TestStack.new
		# Crank
		when 5
			m_currTest = n.TestCrank.new
		# Pulley
		when 6
			m_currTest = n.TestPulley.new
		# Gears
		when 7
			m_currTest = n.TestGears.new
		# Wrap around
		else
			if m_currId < 0 then
				m_currId = 7
				m_currTest = n.TestGears.new
			else
				m_currId = 0
				m_currTest = n.TestBridge.new
			end
		end
	end
	
	# update current test
	m_currTest.Update()
	
	# Update input (last)
	n.Input.update()
	
	# update counter and limit framerate
	n.Main.m_fpsCounter.update()
	n.FRateLimiter.limitFrame(30)
}

n._root.addEventListener(n.Event.ENTER_FRAME, update, false, 0, true)

n.Main.m_fpsCounter.x = 7
n.Main.m_fpsCounter.y = 5
n._root.addChildAt(n.Main.m_fpsCounter, 0)

n.Main.m_sprite = n.Sprite.new
n._root.addChild(n.Main.m_sprite)
# input
m_input = n.Input.new(n.Main.m_sprite)


#Instructions Text
instructions_text = n.TextField.new

instructions_text_format = n.TextFormat.new("Arial", 16, 0xffffff, false, false, false)
instructions_text_format.align = n.TextFormatAlign.RIGHT

instructions_text.defaultTextFormat = instructions_text_format
instructions_text.x = 140
instructions_text.y = 4.5
instructions_text.width = 495
instructions_text.height = 61
instructions_text.text = "Box2DFlashAS3 examples: \n'Left'/'Right' arrows to go to previous/next example. \n'R' to reset."
n._root.addChild(instructions_text)

# textfield pointer
n.Main.m_aboutText = n.TextField.new
m_aboutTextFormat = n.TextFormat.new("Arial", 16, 0x00CCFF, true, false, false)
m_aboutTextFormat.align = n.TextFormatAlign.RIGHT
n.Main.m_aboutText.defaultTextFormat = m_aboutTextFormat
n.Main.m_aboutText.x = 434
n.Main.m_aboutText.y = 71
n.Main.m_aboutText.width = 200
n.Main.m_aboutText.height = 30
n._root.addChild(n.Main.m_aboutText)

# Make a big invisible box to cover the stage so that input focus doesn't change when mousing over the textfields
# (Please let me know if there's a better way to solve this problem) (:
inputFixSprite = n.Sprite.new
inputFixSprite.graphics.lineStyle(0,0,0)
inputFixSprite.graphics.beginFill(0,0)
inputFixSprite.graphics.moveTo(-10000, -10000)
inputFixSprite.graphics.lineTo(10000, -10000)
inputFixSprite.graphics.lineTo(10000, 10000)
inputFixSprite.graphics.lineTo(-10000, 10000)
inputFixSprite.graphics.endFill()
n._root.addChild(inputFixSprite)
