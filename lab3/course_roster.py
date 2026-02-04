"""
ECE387: Course Roster
This program models a university course management system where cadets enroll in courses, receive grades, and calculate GPAs. It uses Python's object-oriented programming concepts, including classes, objects, and methods.

Author: Dr. Stan Baek, United Stated Air Force Academy
Date: 19 Jan 2025

**IMPORTANT DISCLAIMER** This code is intended solely for use within the ECE387 class at the United States Air Force Academy. Unauthorized sharing, distribution, or reproduction of this code is strictly prohibited. 
"""


class Person:
    """
    A class representing a person, with attributes for their name and the list of courses they are associated with.
    """

    def __init__(self, name: str):
        """
        Initialize the Person object with a name and an empty list of courses.
        
        :param name: The name of the person as a string.
        """
        self.name = name
        self.courses: list[Course] = []

    def add_course(self, course) -> None:
        """
        Add a course to the person's list of courses.
        
        :param course: An instance of the Course class.
        :return: None
        """
        self.courses.append(course)

    def list_courses(self) -> str:
        """
        Generate a comma-separated string of the courses the person is taking.
        
        Example: If the person is taking ECE387 and ECE487, it returns "ECE387, ECE487".
        
        :return: String representation of the list of courses.
        """
        return ", ".join(map(str, self.courses))
    
    def __str__(self) -> str:
        """
        Return a string representation of the person, which is their name.
        
        :return: The name of the person.
        """
        return self.name


class Instructor(Person):
    """
    Represents an instructor.

    Attributes:
        name (str): The name of the instructor.
        courses (list): A list of courses the instructor is teaching.
    """

    def __init__(self, name: str):
        """
        Constructor for the Instructor class.
        :param name: The name of the instructor as a string.
        """
        super().__init__(name=name)
        # List of courses the instructor is teaching (initially empty)

    def grade(self, course):
        """
        Print a message indicating that the instructor is grading a course.
        :param course: The course being graded.
        :return: None
        """
        print(f"Dr. {self.name} is grading {course}.")


class Cadet(Person):
    """
    Represents a cadet.

    Attributes:
        name (str): The name of the cadet.
        courses (list): A list of courses the cadet is enrolled in.
    """

    def __init__(self, name: str):
        """
        Constructor for the Cadet class.
        :param name: The name of the cadet as a string.
        """
        super().__init__(name=name)
        # List of courses the cadet is enrolled in (initially empty)

    def march(self):
        """
        Print a message indicating that the cadet is marching.
        :return: None
        """
        print(f"Cadet {self.name} is marching.")
        
    def get_gpa(self) -> float:
        """
        Calculate the cadet's GPA based on the grades and credit hours of their courses.
        GPA = sum(course_grade * course_credit) / total_credits
        :return: The GPA as a float.
        """
        total_points = 0.0  # Total weighted grade points
        total_credits = 0  # Total credit hours

        for course in self.courses:
            # Multiply the grade by the course credit and add to total points
            total_points += course.get_grade(self) * course.credit
            # Add the course credit to the total credit hours
            total_credits += course.credit

        # Compute and return GPA
        return 0 if total_credits == 0 else total_points / total_credits

        
class Course:
    """
    Represents a course in the university system.

    Attributes:
        name (str): The name of the course.
        credit (int): The number of credit hours for the course.
        grades (dict): A dictionary mapping cadets to their grades.
    """

    def __init__(self, name: str, credit: int):
        """
        Constructor for the Course class.
        :param name: The name of the course as a string.
        :param credit: The number of credit hours for the course.
        """
        self.name = name  # Course name
        self.credit = credit  # Credit hours
        self.grades: dict[Cadet, float] = {}  # Dictionary mapping cadets to their grades

    def __str__(self) -> str:
        """
        String representation of the Course class.
        Example: If the course is "ECE245", print(ECE245) will return "ECE245".
        :return: The name of the course.
        """
        return self.name

    def enroll(self, cadets: list) -> None:
        """
        Enroll multiple cadets in this course and initialize their grades to 0.
        :param cadets: A list of Cadet objects to enroll.
        :return: None
        """
        for cadet in cadets:
            self.grades[cadet] = 0  # Initialize the grade to 0 for each cadet
            cadet.add_course(self)  # Add this course to the cadet's list of courses

    def give_grade(self, grades: list) -> None:
        """
        Assign grades to cadets in this course.
        :param grades: A list of [Cadet, grade] pairs, e.g., [[cadet1, 85], [cadet2, 90]].
        :return: None
        """
        for grade in grades:
            self.grades[grade[0]] = grade[1]  # Assign the grade to the corresponding cadet

    def get_course_average(self) -> float:
        """
        Calculate the average grade of all cadets in the course.
        :return: The course average as a float.
        """
        return sum(self.grades.values()) / len(self.grades)  # Compute the average

    def get_roster(self) -> str:
        """
        Generate a comma-separated string of cadets' names enrolled in this course.
        Example: "Peter Parker, Clark Kent, Bruce Wayne".
        :return: String of cadets' names.
        """
        cadet_names = [cadet.name for cadet in self.grades.keys()]  # Extract cadet names
        return ", ".join(cadet_names)  # Join names with commas

    def get_grade(self, cadet) -> float:
        """
        Retrieve the grade of a specific cadet in this course.
        :param cadet: An instance of the Cadet class.
        :return: The grade as an integer.
        """
        return self.grades[cadet]


def main():
    """
    Main function to test the Course and Cadet classes.
    - Creates courses and cadets.
    - Enrolls cadets in courses.
    - Assigns grades and calculates averages and GPAs.
    """

    # Create courses with names and credit hours
    ece333 = Course("ECE333", 3)
    ece382 = Course("ECE382", 3)
    ece387 = Course("ECE387", 3)
    ece487 = Course("ECE487", 3)
    ece499 = Course("ECE499", 1)

    # Create cadets with names
    spiderman = Cadet("Peter Parker")
    superman = Cadet("Clark Kent")
    batman = Cadet("Bruce Wayne")
    blackwidow = Cadet("Natasha Romanova")
    hulk = Cadet("Bruce Banner")

    # Enroll cadets in courses
    ece333.enroll([spiderman, superman, batman, blackwidow])
    ece382.enroll([superman, batman, blackwidow])
    ece387.enroll([spiderman, superman, batman, blackwidow])
    ece487.enroll([superman, blackwidow])
    ece499.enroll([batman, hulk])

    # Store courses and cadets in lists
    courses = [ece333, ece382, ece387, ece487, ece499]
    cadets = [spiderman, superman, batman, blackwidow]

    # Display the course roster for each course
    for course in courses:
        print(f"Cadets enrolled in {course}: {course.get_roster()}")

    # Display the list of courses each cadet is taking
    for cadet in cadets:
        print(f"{cadet} is taking {cadet.list_courses()}")

    # Assign grades to cadets in each course
    ece333.give_grade([[spiderman, 58], [superman, 95], [batman, 73], [blackwidow, 87]])
    ece382.give_grade([[superman, 83], [batman, 82], [blackwidow, 92]])
    ece487.give_grade([[superman, 91], [blackwidow, 98]])
    ece499.give_grade([[batman, 91]])

    # Calculate and display the course average for each course
    for course in courses:
        print(f"The average grade in {course} is {course.get_course_average():.2f}")

    # Calculate and display the GPA for each cadet
    for cadet in cadets:
        print(f"{cadet}'s GPA is {cadet.get_gpa():.2f}")


# Entry point for the program
if __name__ == "__main__":
    main()
