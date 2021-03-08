% Testing types sent from Python 

function output = testTypes(list1)

parsed_list1 = parselist(list1)
parsed_list2 = transpose(parsed_list1)
parsed_list2
output = 1;

    function parsed_list = parselist(list)
        parsed_list = cell2mat(list);
    end

end 
