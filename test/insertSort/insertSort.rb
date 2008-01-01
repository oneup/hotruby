class InsertionSort
	msg = "Insertion Sort:"
	puts msg
	
	def sort
		ary = [3, 6, 2, 5, 3, 7, 1, 8]
		puts "Before insertion sort"
		puts ary

		for i in 1..(ary.length-1) do
			n = i
			while n >= 1 && ary[n] < ary[n-1] do
				if ary[n] < ary[n-1]
					tmp = ary[n]
					ary[n] = ary[n-1]
					ary[n-1] = tmp
				end
				n-=1
			end
		end
		puts "After insertion sort"
		puts ary
	end
end

InsertionSort.new.sort
