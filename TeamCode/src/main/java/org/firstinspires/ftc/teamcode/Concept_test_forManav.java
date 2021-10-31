package org.firstinspires.ftc.teamcode;

public class Concept_test_forManav {
    String s = "new";


    private static String sayHello(String name){
        String helloString= "Hello "+name;
        return helloString;
    }

    public static void main(String[] args){
        //add <option name="delegatedBuild" value="false" /> under .idea\gradle.xml inside GradleProjectSettings
        Concept_test_forManav concept_test_forManav = new Concept_test_forManav();
        System.out.println(Concept_test_forManav.sayHello("Manav"));

    }
}
