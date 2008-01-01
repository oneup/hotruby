class Hoge
	def add_msg &block
		block.yield "World"
	end
end

def main
	pre = "Hello"
	block2 = new Proc(do
		space = " "
		Hoge.new.add_msg do |msg|
			fuga = "!"
			puts pre + space + msg + fuga
		end
	end)
end

main
