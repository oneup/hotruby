class Hoge
	def add_msg &block
		block.yield "World"
	end
end

class Foo
	def main
		pre = "Hello"
		@space = " "
		Hoge.new.add_msg do |msg|
			fuga = "!"
			puts pre + @space + msg + fuga
		end
	end
end

Foo.new.main
