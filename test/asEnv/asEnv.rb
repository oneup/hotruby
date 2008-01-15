n = $native
n.import("flash.text.*")
n.import("flash.system.*")

textField = n.TextField.new
textField.autoSize = n.TextFieldAutoSize.LEFT
textField.text = "Memory: " + n.System.totalMemory.to_s + " bytes\n"
n._root.addChild(textField)

g = n._root.graphics
g.beginFill 0xFF0000
g.drawCircle 100, 100, 100
g.endFill

g.beginFill 0x00FF00, 0.5
g.drawCircle 200, 200, 100
g.endFill

callback = Proc.new {|evt| textField.text += "type: " + evt.type + "\n" }
textField.addEventListener "click", callback
