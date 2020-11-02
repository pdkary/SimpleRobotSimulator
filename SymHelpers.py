import sympy as sym

TOLERANCE = 10**-5

def flatten_syms(expression,expr_type):
    output = []
    exp_tuple = expression.as_two_terms()
    for x in exp_tuple:
        if type(x) == sym.Add:
            output+=[expr_type,flatten_syms(x,"+")]
        elif type(x) == sym.Mul:
            output+=[expr_type,flatten_syms(x,"*")]
        else:
            if type(x) == sym.Symbol or type(x) == sym.cos or type(x) == sym.sin:
                new_x = x
            else:
                new_x = x if abs(x) > TOLERANCE else 0
            output+=[new_x,expr_type]

    if output[-1] == "*" or output[-1]=="+":
        output=output[0:-1]
    if output[0] == "+" or output[0]=="*":
        output=output[1:]
    if (output[1] == "+" and output[2] == "+") or (output[1]=="*" and output[2]=="*"):
        del output[1]
    if len(output) == 2:
        output = [output[0],expr_type,output[1]]
    return output

def evaluate_flattened_sym(flat_sym):
    val1 = flat_sym[0]
    operator = flat_sym[1]
    val2 = flat_sym[2]
    if type(val1) != list and type(val2) != list:
        return val1*val2 if operator == "*" else val1+val2
    elif type(val1) == list and type(val2) != list:
        return evaluate_flattened_sym(val1)*val2 if operator == "*" else evaluate_flattened_sym(val1)+val2
    elif type(val1) != list and type(val2) == list:
        return val1* evaluate_flattened_sym(val2) if operator == "*" else val1+evaluate_flattened_sym(val2)
    elif type(val1)==list and type(val2)==list:
        return evaluate_flattened_sym(val1)*evaluate_flattened_sym(val2) if operator == "*" else evaluate_flattened_sym(val1)+evaluate_flattened_sym(val2)

def round_sym_mat(symmatrix,n):
    for i in range(len(symmatrix)):
        val = symmatrix[i]
        if type(val) == sym.Add:
            flat_expr = flatten_syms(val,"+")
            # print(flat_expr)
            newval = evaluate_flattened_sym(flat_expr)
            # print("efs returned:\t",newval)
            symmatrix[i] = newval
        if type(val) == sym.Mul:
            flat_expr = flatten_syms(val,"*")
            # print(flat_expr)
            newval = evaluate_flattened_sym(flat_expr)
            # print("efs returned:\t",newval)
            symmatrix[i]=newval

        if type(val) == sym.Float:
            if abs(val) < TOLERANCE:
                symmatrix[i] = 0
        
    return sym.N(symmatrix,n)

