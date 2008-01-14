textField = $native.TextField.new
textField.autoSize = $native.TextFieldAutoSize.LEFT
textField.text = "Memory: " + $native.System.totalMemory.to_s + " bytes"
$native._root.addChild(textField)

g = $native._root.graphics
g.beginFill 0xFF0000
g.drawCircle 100, 100, 100
g.endFill

g.beginFill 0x00FF00, 0.5
g.drawCircle 200, 200, 100
g.endFill

callback = Proc.new { textField.text += "clicked" }
textField.addEventListener "click", callback
