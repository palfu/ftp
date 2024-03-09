#include <vector>
#include <algorithm>
#include <unordered_set>
#include <stdlib.h>
#include <stdio.h>
using namespace std;

class Solution
{
public:
    vector<vector<int>> threeSum(vector<int>& nums)
    {
        int num_size = nums.size();
        std::vector<vector<int>> res;
        if (num_size < 3) {
            return res;
        }
        std::sort(nums.begin(), nums.end());
        std::unordered_set<int> query_set(nums.begin(), nums.end());
        int prev_i = nums[0] - 1;
        for (int i = 0; i < num_size; i++) {
            if (nums[i] == prev_i) {
                continue;
            }
            prev_i     = nums[i];
            int prev_j = nums[i] - 1;
            for (int j = i + 1; j < num_size; j++) {
                if (prev_j == nums[j]) {
                    continue;
                }
                prev_j  = nums[j];
                int val = -nums[i] - nums[j];
                if (query_set.find(val) != query_set.end()) {
                    std::vector<int> tmp = {nums[i], nums[j], val};
                    res.emplace_back(tmp);
                }
            }
        }

        return res;
    }
};

std::vector<int> randVector(int num, int max_val = 100)
{
    std::vector<int> vec;
    for (int i = 0; i < num; i++) {
        int val = static_cast<int>(random() % (2 * max_val)) - max_val;
        vec.emplace_back(val);
    }
    return vec;
}

int main(int argc, char** argv)
{
    auto vec = randVector(100);

    Solution solution;
    auto res = solution.threeSum(vec);
    for (auto& tmp : res) {
        std::sort(tmp.begin(), tmp.end());
        printf("find res %d %d %d\n", tmp[0], tmp[1], tmp[2]);
    }

    return 0;
}