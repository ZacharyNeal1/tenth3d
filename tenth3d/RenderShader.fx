struct VS_IN
{
    float4 pos : POSITION;
    float4 col : COLOR;
};

struct PS_IN
{
	float4 pos : SV_POSITION;
    float4 col : COLOR;
};

float4x4 worldViewProj;

//Texture2D g_texture : register(t0);
//SamplerState g_sampler : register(s0);

PS_IN VS(VS_IN input)
{
	PS_IN output = (PS_IN) 0;
	
	output.pos = mul(input.pos, worldViewProj);
    output.col = float4(1-clamp((output.pos.z / 400.0), 0.0, 1.0), 0.2, 0.2, 1.0);
	
	return output;
}

float4 PS(PS_IN input) : SV_Target
{
    return  /*float4(1.0f, 1.0f, 1.0f, 1.0f);*/ input.col; //g_texture.Sample(g_sampler, float2(input.col.x, input.col.y) );

}