startTime = Time.new.to_f

sum = ""
50000.times{|e| sum += e.to_s}

endTime = Time.new.to_f
t = (endTime - startTime).to_s
puts t + ' sec'
