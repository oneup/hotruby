class Pi
	def initialize hoge, hoge2
		@a = 352.0 + hoge + hoge2
		@@c = 3
	end
	def getA
		@a
	end
end
	
class Pi2 < Pi
	def initialize hoge, hoge2
		super
		@b = 110.0 + @@c
	end
	def calc
		getA / @b
	end
end

puts '335/113 = ' + Pi2.new(1, 2).calc
