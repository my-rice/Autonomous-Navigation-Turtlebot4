import math

def compute_next_angle(current_angle, result):
        if result == "right":
            current_angle -= 90
        elif result == "left":
            current_angle += 90
        elif result == "goback":
            current_angle += 180

        if current_angle >= 360:
            current_angle -= 360
        if current_angle < 0:
            current_angle += 360
        return current_angle

def get_intersection_points(next_goal, theta, result):
    x1, y1 = next_goal

    # Parametri del cerchio
    rho = 4
    xc, yc = next_goal

    # Convert frame angle to standard frame (remove the offset)
    standard_angle = theta + 90
    standard_angle = compute_next_angle(standard_angle, result)

    # Portiamo l'angolo tra 0 e 360 gradi
    angle = standard_angle - 90
    if angle < 0:
        angle += 360
    if angle >= 360:
        angle -= 360

    if standard_angle == 90:
        x_intersect1 = xc
        y_intersect1 = yc + rho
        x_intersect2 = xc
        y_intersect2 = yc - rho
        intersection_points = [(x_intersect1, y_intersect1), (x_intersect2, y_intersect2)]
        return intersection_points, angle

    m = math.tan(math.radians(standard_angle))
    q = y1 - m * x1

    # Coefficienti dell'equazione quadratica
    A = 1 + m**2
    B = 2 * (m * q - m * yc - xc)
    C = xc**2 + yc**2 + q**2 - 2 * yc * q - rho**2

    # Risolvi l'equazione quadratica per trovare x
    delta = B**2 - 4 * A * C

    if delta < 0:
        raise ValueError("No intersection points")

    x_intersect1 = (-B + math.sqrt(delta)) / (2 * A)
    x_intersect2 = (-B - math.sqrt(delta)) / (2 * A)

    # Calcola i punti di intersezione corrispondenti su y
    y_intersect1 = m * x_intersect1 + q
    y_intersect2 = m * x_intersect2 + q

    # Riportare i punti nel sistema originale (x verso l'alto, y verso sinistra)
    intersection_points = [(x_intersect1,-y_intersect1), (x_intersect2, -y_intersect2)]

    return intersection_points, angle

# Esempio di utilizzo
next_goal = (-3.5, -9.5)
theta = 0.00018
result = "right"  # Supponiamo che non influisca sull'angolo calcolato

intersection_points, angle = get_intersection_points(next_goal, theta, result)
print("Punti di intersezione:", intersection_points)
print("Angolo:", angle)
