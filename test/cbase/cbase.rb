obj = Object.new
class << obj
	def test
	     puts "Hello World! 1"
	end
end
obj.test

def obj.test
	puts "Hello World! 2"
end
obj.test
