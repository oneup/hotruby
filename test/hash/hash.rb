def printHash hash
	puts hash['a'] + hash['b'] + hash['c']
end
v = {'a' => "Create ", 'b' => 'from '}
v['c'] = 'Hash'
printHash v
puts v.length
