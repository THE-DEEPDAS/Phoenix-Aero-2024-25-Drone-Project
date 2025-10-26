
vector<int> getResources(int inc, int dec, vector<int> performance) {
  int n=  performance.size();
  vector<int> rank(n);
unordered_map <int,int> freq;
vector<int> answer;
for( auto & num:performance)
{
    freq[num]++;
}
for(int i=0;i<n;i++)
{ int cnt=0;
    for(const auto &ele: freq)
    {
        if (ele.first >= performance[i])
        {
            cnt++;
        }
    }
    rank[i]=cnt;
  answer.push_back(inc*(n+1-cnt)-(dec*freq[i])); 
}
 

return answer;

}
