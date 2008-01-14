textField = $native.TextField.new
textField.autoSize = $native.TextFieldAutoSize.LEFT
textField.text = "Memory: " + $native.System.totalMemory.to_s + " bytes\n"
$native._root.addChild(textField)

g = $native._root.graphics
g.beginFill 0xFF0000
g.drawCircle 100, 100, 100
g.endFill

g.beginFill 0x00FF00, 0.5
g.drawCircle 200, 200, 100
g.endFill

callback = Proc.new {|evt| textField.text += "type: " + evt.type + "\n" }
textField.addEventListener "click", callback
