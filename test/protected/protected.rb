class Protected
	def test
		puts "Hello World!"
	end
	protected :test
end

Protected.new.test