def decision_making(data_list):
    count_red, count_blue, count_none = 0, 0, 0
    best_number, best_confidence = None, -1
    
    for entry in data_list:
        red, blue, none, num, confidence = entry
        
        count_red += red
        count_blue += blue
        count_none += none
        
        if num != -1 and confidence > best_confidence:
            best_number, best_confidence = num, confidence
    
    print(f"Red: {count_red}, Blue: {count_blue}, None: {count_none}, Best Number: {best_number} (Confidence: {best_confidence})")
    
    if count_red > count_blue:
        decision = "Red Dominates"
    elif count_blue > count_red:
        decision = "Blue Dominates"
    else:
        decision = "No clear winner"
    
    return {"decision": decision, "best_number": best_number, "confidence": best_confidence}

# Ejemplo de uso
data_list = [[1, 0, 0, 8, 10.4], [0, 1, 0, 5, 8.7], [1, 0, 0, 3, 12.1]]
result = decision_making(data_list)
print(result)
