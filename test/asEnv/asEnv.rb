$n = $native
$n.import("flash.text.*")
$n.import("flash.system.*")

class ASEnv 
	def initialize
		showMemory
		drawCircle
		addButton
	end

	# TextField for memory
	def showMemory
		@textField = $n.TextField.new
		@textField.autoSize = $n.TextFieldAutoSize.LEFT
		@textField.text = "Memory: " + $n.System.totalMemory.to_s + " bytes\n"
		@textField.x = 400
		$n._root.addChild(@textField)
	end
	
	def drawCircle
		g = $n._root.graphics
		g.beginFill 0xFF0000
		g.drawCircle 100, 100, 100
		g.endFill

		g.beginFill 0x00FF00, 0.5
		g.drawCircle 200, 200, 100
		g.endFill
	end
	
	def addButton
		callback = Proc.new {|evt| @textField.text += "type: " + evt.type + "\n" }
		@textField.addEventListener "click", callback
	end
end

ASEnv.new
