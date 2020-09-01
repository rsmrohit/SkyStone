package org.firstinspires.ftc.teamcode;

import com.sun.org.apache.xpath.internal.operations.String;

public class TestingWorld {
    enum names{
        p1(new person(1)), p2(new person(2));

        private person p;

        private names(person p){
            this.p = p;
        }
    }

    public static class person {
        int num;

        person(int number) {
            this.num = number;
        }

        public void printNum() {
            System.out.println(this.num);
        }
    }

    public static void main(String args[]){
        for (names name : names.values()){
            name.p.printNum();
        }
    }
}
